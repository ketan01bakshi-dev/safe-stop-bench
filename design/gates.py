"""Human approval gates, bound to artifact content (from the HIL ML Ops pipeline's gates/human.py; here since v2.14).

An approval records the sha256 of exactly what the reviewer saw. If the artifact changes afterwards
(new draft, edited basis, a new bench run), the approval no longer matches and the gate re-opens. Every decision
is appended to audit.jsonl with who, when, which hash, and why.

Flow for a live run:
  1. Pipeline reaches a gate -> writes gates/pending_<gate>.md (review packet) -> stops (exit code 3).
  2. Reviewer: python -m design approve --campaign <c> --gate <gate> --by "<name>" [--comment ...]
               or reject --reason "..."  (the reason is kept and shown in the next review packet)
  3. Re-run the campaign. The rules drafter is deterministic, so the approved hash matches.
"""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class GatePending(PermissionError):
    def __init__(self, gate: str, review_path: Path, reason: str = "") -> None:
        self.gate = gate
        self.review_path = review_path
        msg = f"Human gate '{gate}' awaiting approval. Review {review_path}"
        if reason:
            msg += f" ({reason})"
        super().__init__(msg)


def artifact_hash(artifacts: Any) -> str:
    raw = json.dumps(artifacts, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


AGENT_AUTO = "agent-auto:"   # prefix of the approver recorded when the agent approves for the owner (autonomy test)
AUTONOMOUS_APPROVER = "claude on Ketan's behalf (test 2026-10-09)"


class HumanGate:
    # safe-stop gates: the test basis (the oracle), the derived cases (before bench time), the results + defects (before the tracker)
    GATE_BASIS = "basis_review"
    GATE_CASES = "case_review"
    GATE_PUBLISH = "publish_approval"
    ALL = (GATE_BASIS, GATE_CASES, GATE_PUBLISH)

    def __init__(
        self,
        campaign_dir: Path,
        approvals: dict[str, bool] | None = None,
        *,
        auto_approve: bool = False,
        approver: str = "cli",
        auto_actor: str = "demo-auto",
    ) -> None:
        self.campaign_dir = campaign_dir
        self.approvals = dict(approvals or {})  # per-run CLI approvals (--approve <gate>)
        self.auto_approve = auto_approve
        self.approver = approver
        # Who an automatic approval is recorded as: "demo-auto" (tests) or AGENT_AUTO + a label (autonomy test, 2026-10-09).
        self.auto_actor = auto_actor
        self.audit_path = campaign_dir / "audit.jsonl"
        self.store_path = campaign_dir / "gates" / "gates.json"
        self.decisions: dict[str, dict[str, Any]] = {}  # gate -> decision taken during this run

    # ---- persistence ------------------------------------------------------------------------

    def load(self) -> dict[str, dict[str, Any]]:
        if self.store_path.exists():
            return json.loads(self.store_path.read_text(encoding="utf-8"))
        return {}

    def _save(self, gate: str, record: dict[str, Any]) -> None:
        data = self.load()
        record.setdefault("feedback", (data.get(gate) or {}).get("feedback", ""))
        data[gate] = record
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        self.store_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        self.decisions[gate] = record

    # ---- gate API ---------------------------------------------------------------------------

    def is_approved(self, gate: str, artifacts: Any = None) -> bool:
        rec = self.load().get(gate) or {}
        return rec.get("decision") == "approved" and rec.get("artifact_hash") == artifact_hash(artifacts)

    def require(self, gate: str, artifacts: Any = None, summary_md: str = "") -> dict[str, Any]:
        h = artifact_hash(artifacts)
        rec = self.load().get(gate) or {}
        # Automatic approvals (tests, or the agent acting for the owner in an autonomy test) are never reused by a run
        # that has human gates switched on: only a person's approval carries over.
        auto_record = str(rec.get("approver", "")) == "demo-auto" or str(rec.get("approver", "")).startswith(AGENT_AUTO)
        demo_record = auto_record and not self.auto_approve
        if rec.get("decision") == "approved" and rec.get("artifact_hash") == h and not demo_record:
            self.decisions[gate] = rec
            self._log("reuse_approval", gate, rec.get("approver", "?"), {"artifact_hash": h[:16]})
            return rec
        if self.auto_approve or self.approvals.get(gate):
            automatic = self.auto_approve and not self.approvals.get(gate)
            actor = self.auto_actor if automatic else self.approver
            comment = ("approved via --approve" if not automatic else
                       "auto-approved (demo mode)" if actor == "demo-auto" else
                       "auto-approved by the agent on the owner's behalf (autonomy test; no human review)")
            return self._decide(gate, "approved", h, actor, comment)

        reason = ""
        if rec.get("decision") == "approved":
            reason = "artifact changed since it was approved"
        review = self._write_review_packet(gate, artifacts, h, summary_md, rec)
        self._save(gate, {"decision": "pending", "artifact_hash": h,
                          "at": _now(), "review_packet": str(review),
                          "previous": {k: rec.get(k) for k in ("decision", "approver", "comment")} if rec else None})
        self._log("pending", gate, "system", {"artifact_hash": h[:16], "reason": reason})
        raise GatePending(gate, review, reason)

    def approve(self, gate: str, approver: str | None = None, comment: str = "",
                artifact_hash_value: str | None = None) -> dict[str, Any]:
        """Approve the pending artifact (or, with no pending record, the empty artifact)."""
        rec = self.load().get(gate) or {}
        h = artifact_hash_value or rec.get("artifact_hash") or artifact_hash(None)
        return self._decide(gate, "approved", h, approver or self.approver, comment)

    def reject(self, gate: str, approver: str, reason: str) -> dict[str, Any]:
        rec = self.load().get(gate) or {}
        record = {"decision": "rejected", "artifact_hash": rec.get("artifact_hash", ""), "approver": approver,
                  "comment": reason, "at": _now(),
                  "feedback": "\n".join(filter(None, [rec.get("feedback", ""), f"- {reason}"]))}
        self._save(gate, record)
        self._log("rejected", gate, approver, {"artifact_hash": record["artifact_hash"][:16], "comment": reason})
        return record

    def feedback(self, gate: str) -> str:
        """Accumulated rejection reasons for this gate, fed into every later draft.

        Kept on the record through later pending/approved states on purpose: the approved draft was
        generated *with* this feedback, so re-runs must use the same prompt to reproduce it.
        """
        return (self.load().get(gate) or {}).get("feedback", "")

    def _decide(self, gate: str, decision: str, h: str, approver: str, comment: str) -> dict[str, Any]:
        record = {"decision": decision, "artifact_hash": h, "approver": approver,
                  "comment": comment, "at": _now()}
        self._save(gate, record)
        self._log(decision, gate, approver, {"artifact_hash": h[:16], "comment": comment})
        return record

    def _write_review_packet(self, gate: str, artifacts: Any, h: str, summary_md: str,
                             previous: dict[str, Any]) -> Path:
        path = self.campaign_dir / "gates" / f"pending_{gate}.md"
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            f"# Review required: `{gate}`",
            "",
            f"- Campaign dir: `{self.campaign_dir}`",
            f"- Artifact hash: `{h}`",
            f"- Previous decision: {previous.get('decision', 'none')}"
            + (f" by {previous.get('approver')}: {previous.get('comment')}" if previous.get("approver") else ""),
            "",
            f"Approve:  `python -m design approve --campaign {self.campaign_dir.name} --gate {gate} --by \"<name>\"`",
            f"Reject:   `python -m design reject --campaign {self.campaign_dir.name} --gate {gate}"
            + " --by \"<name>\" --reason \"<what to fix>\"`",
            "",
        ]
        if summary_md:
            lines += [summary_md, ""]
        lines += ["## Exact artifacts under review", "```json",
                  json.dumps(artifacts, indent=2, default=str), "```"]
        path.write_text("\n".join(lines), encoding="utf-8")
        return path

    # ---- audit ------------------------------------------------------------------------------

    def _log(self, action: str, gate: str, actor: str, extra: dict[str, Any] | None = None) -> None:
        entry = {"timestamp": _now(), "action": action, "gate": gate, "actor": actor, **(extra or {})}
        self.audit_path.parent.mkdir(parents=True, exist_ok=True)
        with self.audit_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry) + "\n")

    def log_eval(self, stage: str, metrics: dict[str, Any]) -> None:
        self._log("eval", stage, "system", {"metrics": metrics})


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()
