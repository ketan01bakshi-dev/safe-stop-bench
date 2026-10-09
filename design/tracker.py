"""Defect trackers, both free: a local JSON file (default, nothing leaves the PC) and GitHub Issues in one repository
(--tracker github --github-repo owner/name; uses the GitHub CLI login). Jira Cloud was dropped on 2026-10-09 (cost); the
earlier client is in git history (v2.15, design/jira.py).

Same rule as HIL ML Ops: a failure gets a NEW defect only when no open defect has the same requirement AND the same
failure signature (`sig-<fault>` + `mode-<check>` labels). Otherwise this run's evidence is added as a comment, so a
recurring failure piles evidence onto one ticket instead of opening duplicates.
"""
from __future__ import annotations

import json
import re
import subprocess
import urllib.parse
from datetime import datetime, timezone
from pathlib import Path

LABEL = "ssb"   # every safe-stop defect carries it, so searches never mix with other projects' tickets


class LocalTracker:
    """Issues in one JSON file (reports/design/tracker.json), shared by all campaigns so de-duplication works across them."""

    def __init__(self, path: Path):
        self.path = path
        self.name = f"local tracker {path.name}"

    def _load(self) -> dict:
        return json.loads(self.path.read_text(encoding="utf-8")) if self.path.exists() else {"next": 1, "issues": {}}

    def _save(self, d: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(d, indent=1), encoding="utf-8")

    def open_defects(self, req_id: str) -> list[dict]:
        return [{"key": k, "title": i["title"], "labels": i["labels"]} for k, i in self._load()["issues"].items()
                if i["status"] != "done" and {LABEL, "defect", req_id} <= set(i["labels"])]

    def create_issue(self, title: str, description: str, labels: list[str]) -> dict:
        d = self._load()
        key = f"SSB-{d['next']}"
        d["next"] += 1
        d["issues"][key] = {"title": title, "description": description, "labels": labels, "status": "open",
                            "created": _now(), "comments": []}
        self._save(d)
        return {"key": key, "url": str(self.path)}

    def add_comment(self, key: str, text: str) -> dict:
        d = self._load()
        d["issues"][key]["comments"].append({"at": _now(), "text": text})
        self._save(d)
        return {"key": key, "url": str(self.path)}


class GitHubError(RuntimeError):
    pass


def gh_api(method: str, path: str, body: dict | None = None):
    """GitHub REST through the GitHub CLI's stored login (`gh auth login`); no token is read or kept here."""
    cmd = ["gh", "api", "-X", method, path, "-H", "Accept: application/vnd.github+json"] + (["--input", "-"] if body is not None else [])
    try:
        proc = subprocess.run(cmd, input=json.dumps(body) if body is not None else None, capture_output=True, text=True,
                              encoding="utf-8", timeout=90, check=False)
    except FileNotFoundError as e:
        raise GitHubError("the GitHub CLI 'gh' is not installed or not on PATH (https://cli.github.com, then 'gh auth login')") from e
    if proc.returncode != 0:
        try:
            msg = json.loads(proc.stdout).get("message", "")
        except (json.JSONDecodeError, AttributeError):
            msg = ""
        raise GitHubError(f"{method} {path} -> {msg or (proc.stderr or proc.stdout).strip()[:300]}")
    return json.loads(proc.stdout) if proc.stdout.strip() else {}


class GitHubTracker:
    """Defects as GitHub Issues in one repository (free, private repos included). Same three methods as LocalTracker, so the
    duplicate rule is unchanged: labels `ssb`, `defect`, the requirement ID, `sig-<fault>`, `mode-<check>`. Keys are `GH-<n>`.
    Only called after the publish_approval gate. `api` is injectable for tests."""

    def __init__(self, repo: str, api=None):
        if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repo or ""):
            raise GitHubError(f"github repo must look like owner/name, got {repo!r}")
        self.repo, self.api = repo, api or gh_api
        self.name = f"GitHub Issues {repo}"

    @staticmethod
    def _label(x: str) -> str:
        return re.sub(r"\s+", "-", x.replace(",", "-")).strip("-")[:50]   # GitHub: at most 50 characters, no commas

    def open_defects(self, req_id: str) -> list[dict]:
        out, page = [], 1
        want = urllib.parse.quote(",".join([LABEL, "defect", self._label(req_id)]))
        while True:
            rows = self.api("GET", f"repos/{self.repo}/issues?state=open&labels={want}&per_page=100&page={page}", None)
            out += [{"key": f"GH-{i['number']}", "title": i.get("title", ""), "labels": [lab["name"] for lab in i.get("labels", [])]}
                    for i in rows if "pull_request" not in i]       # the issues endpoint also lists pull requests
            if len(rows) < 100:
                return out
            page += 1

    def create_issue(self, title: str, description: str, labels: list[str]) -> dict:
        body = {"title": title[:250], "body": description + "\n\n---\nFiled by the safe-stop bench design pipeline after publish_approval.",
                "labels": list(dict.fromkeys(self._label(x) for x in labels if self._label(x)))}
        made = self.api("POST", f"repos/{self.repo}/issues", body)
        return {"key": f"GH-{made['number']}", "url": made.get("html_url", "")}

    def add_comment(self, key: str, text: str) -> dict:
        m = re.fullmatch(r"(?:GH-|#)?(\d+)", key.strip())
        if not m:
            raise GitHubError(f"'{key}' is not a GitHub issue key (expected GH-<number>)")
        made = self.api("POST", f"repos/{self.repo}/issues/{m.group(1)}/comments", {"body": text})
        return {"key": f"GH-{m.group(1)}", "url": made.get("html_url", f"https://github.com/{self.repo}/issues/{m.group(1)}")}


def read_env_file(path: Path) -> dict:
    """KEY=value lines of an env file (read, never copied or logged); used for ANTHROPIC_API_KEY."""
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip().strip('"').strip("'")
    return out


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")
