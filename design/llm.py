"""LLM calls for drafting: Claude Sonnet 5.5 at medium effort, the only model (Ketan, 2026-10-09). Two routes to it:

  subscription (DEFAULT)  the Claude Code CLI signed in to Ketan's Claude plan: counts against the plan's usage limits, no
                          per-token charge. One-time login: `claude auth login`.
  api                     the Anthropic API with ANTHROPIC_API_KEY (+ ANTHROPIC_WORKSPACE_ID for a user-scoped key): billed per
                          token to the API account, a separate wallet from the subscription.

There is NO automatic fallback from one route to the other: a failure (not logged in, usage limit, no key, 401, refusal,
truncation) stops the run with a message, because a silent fallback would spend API money. Both routes send no temperature
(a non-default value is a 400 on Sonnet 5.5), no prefill and no forced tool choice; JSON comes from the prompt and the
drafter's own checks. Every answer is cached (reports/design/.llm_cache/, keyed by model + effort + prompt, the same for both
routes), so a re-run returns the same draft and an approved gate's hash still matches.
"""
from __future__ import annotations

import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

MODEL = "claude-sonnet-5-5"
EFFORT = "medium"
MAX_TOKENS = 16000
PLANS = {"subscription": "claude-code", "api": "anthropic"}
SYSTEM_FALLBACK = ("You are a precise drafting engine inside an automated pipeline. Follow the user's instructions exactly. "
                   "When asked for JSON, answer with the JSON object only. Do not use tools and do not ask questions.")


class LLMUnavailable(RuntimeError):
    pass


def claude_code_binary() -> str | None:
    """env CLAUDE_CODE_BIN, else `claude` on PATH, else the copy bundled with the Claude desktop app (newest version)."""
    explicit = os.environ.get("CLAUDE_CODE_BIN", "").strip()
    if explicit and Path(explicit).exists():
        return explicit
    if shutil.which("claude"):
        return shutil.which("claude")
    shared = Path(__file__).resolve().parents[3] / "tools" / "claude-code" / "claude.exe"   # D:/Agents/tools/claude-code/
    if shared.exists():
        return str(shared)
    appdata = os.environ.get("APPDATA", "")
    found = glob.glob(os.path.join(appdata, "Claude", "claude-code", "*", "*", "claude.exe")) if appdata else []

    def version(p: str) -> tuple[int, ...]:
        m = re.search(r"claude-code[\\/]([\d.]+)", p)
        return tuple(int(x) for x in m.group(1).split(".")) if m else (0,)
    return max(found, key=version) if found else None


def cli_env() -> dict[str, str]:
    """The CLI must use the logged-in plan, never an API key (it would bill the API wallet): drop every ANTHROPIC_* variable and
    the markers of an enclosing Claude Code session."""
    drop = ("ANTHROPIC_", "CLAUDE_CODE_", "CLAUDE_AGENT_SDK", "CLAUDE_PREVIEW", "CLAUDE_EFFORT", "CLAUDE_PID", "CLAUDECODE")
    return {k: v for k, v in os.environ.items() if not k.upper().startswith(drop)}


def parse_cli_output(stdout: str, returncode: int, stderr: str) -> tuple[str, dict, dict]:
    """`claude -p --output-format json` -> (text, tokens, notes). Failures raise LLMUnavailable with the reason."""
    try:
        data = json.loads(stdout) if stdout.strip() else {}
    except json.JSONDecodeError:
        data = {}
    if isinstance(data, list):
        data = next((e for e in reversed(data) if isinstance(e, dict) and e.get("type") == "result"), {})
    text = str(data.get("result") or "")
    blob = (text + " " + (stderr or "")).lower()
    if returncode != 0 or data.get("is_error") or not data:
        if any(w in blob for w in ("login", "not logged in", "authenticat", "oauth", "401")):
            raise LLMUnavailable("Claude Code is not logged in to your plan: run `claude auth login` (claude.ai account)")
        if any(w in blob for w in ("usage limit", "rate limit", "limit reached", "overloaded", "429")):
            raise LLMUnavailable(f"Claude plan usage limit or rate limit reached: {text[:160] or (stderr or '')[:160]}")
        raise LLMUnavailable(f"claude -p failed (exit {returncode}): {(text or stderr or stdout)[:300]}")
    usage = data.get("usage") or {}
    tokens = {"in": int(usage.get("input_tokens", 0)) + int(usage.get("cache_read_input_tokens", 0))
              + int(usage.get("cache_creation_input_tokens", 0)), "out": int(usage.get("output_tokens", 0))}
    return text, tokens, {"api_equivalent_usd": round(float(data.get("total_cost_usd") or 0.0), 6)}


class Client:
    def __init__(self, plan: str, env: dict, cache_dir: Path, sdk=None, runner=None):
        if plan not in PLANS:
            raise ValueError(f"unknown LLM plan {plan!r}; choose one of {sorted(PLANS)}")
        self.plan, self.env, self.cache_dir, self._sdk = plan, env, cache_dir, sdk    # sdk / runner: tests inject fakes
        self._runner = runner or subprocess.run
        self.calls: list[dict] = []   # provider, model, tokens, cached, seconds: for the campaign's usage log

    # ---- subscription route ---------------------------------------------------------------------------------

    def _chat_subscription(self, system: str, turns: list[dict]) -> tuple[str, dict, dict]:
        exe = claude_code_binary()
        if not exe:
            raise LLMUnavailable("Claude Code CLI not found: install Claude Code or set CLAUDE_CODE_BIN")
        # `claude -p` takes one prompt: earlier turns (a repair round) are laid out as a transcript, the last user turn is the ask
        if len(turns) == 1:
            prompt = turns[0]["content"]
        else:
            prompt = "\n\n".join(f"[{t['role'].upper()}]\n{t['content']}" for t in turns) + "\n\n[ASSISTANT]\n"
            prompt = ("Continue this conversation as the ASSISTANT: answer the last USER message.\n\n" + prompt)
        args = [exe, "-p", "--model", MODEL, "--effort", EFFORT, "--tools", "", "--no-session-persistence",
                "--output-format", "json", "--system-prompt", system or SYSTEM_FALLBACK]
        with tempfile.TemporaryDirectory(prefix="ssb_claude_") as cwd:   # neutral directory: no project CLAUDE.md in the prompt
            try:
                proc = self._runner(args, input=prompt, capture_output=True, text=True, encoding="utf-8", cwd=cwd,
                                    env=cli_env(), timeout=600, check=False)
            except subprocess.TimeoutExpired as e:
                raise LLMUnavailable("claude -p timed out after 600 s") from e
        return parse_cli_output(proc.stdout, proc.returncode, proc.stderr)

    # ---- API route --------------------------------------------------------------------------------------------

    def _client(self):
        if self._sdk is None:
            key = self.env.get("ANTHROPIC_API_KEY")
            if not key:
                raise LLMUnavailable("ANTHROPIC_API_KEY is not set (environment or --env-file) for --llm api")
            try:
                import anthropic
            except ImportError as e:
                raise LLMUnavailable("the 'anthropic' package is not installed (pip install anthropic)") from e
            # A user-scoped key (sk-ant-usr-...) is not tied to a workspace: the API then needs the workspace ID as a header.
            workspace = (self.env.get("ANTHROPIC_WORKSPACE_ID") or "").strip()
            self._sdk = anthropic.Anthropic(api_key=key, timeout=300, max_retries=2,   # the SDK retries 408/409/429/5xx itself
                                            default_headers={"anthropic-workspace-id": workspace} if workspace else None)
        return self._sdk

    def _chat_api(self, system: str, turns: list[dict]) -> tuple[str, dict, dict]:
        client = self._client()
        import anthropic
        try:
            msg = client.messages.create(model=MODEL, max_tokens=MAX_TOKENS, output_config={"effort": EFFORT},
                                         **({"system": system} if system else {}), messages=turns)
        except anthropic.AuthenticationError as e:
            raise LLMUnavailable("Anthropic rejected the API key (401)") from e
        except anthropic.RateLimitError as e:
            raise LLMUnavailable(f"rate limited (429) after the SDK's retries: {e.message}") from e
        except anthropic.APIConnectionError as e:
            raise LLMUnavailable(f"Anthropic unreachable: {e.__class__.__name__}") from e
        except anthropic.APIStatusError as e:
            raise LLMUnavailable(f"Anthropic API {e.status_code}: {str(e.message)[:300]}") from e
        if msg.stop_reason == "refusal":
            cat = getattr(getattr(msg, "stop_details", None), "category", None)
            raise LLMUnavailable(f"Claude declined the request (refusal, category {cat})")
        if msg.stop_reason == "max_tokens":
            raise LLMUnavailable("Claude hit max_tokens before finishing")
        text = "".join(b.text for b in msg.content if b.type == "text")
        return text, {"in": msg.usage.input_tokens, "out": msg.usage.output_tokens}, {}

    # ---- both -----------------------------------------------------------------------------------------------------

    def chat(self, messages: list[dict]) -> str:
        provider = PLANS[self.plan]
        key = hashlib.sha256(json.dumps([MODEL, EFFORT, messages], sort_keys=True).encode()).hexdigest()   # same for both routes
        cached = self.cache_dir / f"{key}.json"
        if cached.exists():
            hit = json.loads(cached.read_text(encoding="utf-8"))
            self.calls.append({"provider": provider, "model": MODEL, "cached": True, "tokens": hit.get("tokens")})
            return hit["text"]
        system = "\n\n".join(m["content"] for m in messages if m["role"] == "system")
        turns = [m for m in messages if m["role"] != "system"]
        t0 = time.time()
        text, tokens, notes = (self._chat_subscription if self.plan == "subscription" else self._chat_api)(system, turns)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        cached.write_text(json.dumps({"provider": provider, "model": MODEL, "effort": EFFORT, "text": text, "tokens": tokens}), encoding="utf-8")
        self.calls.append({"provider": provider, "model": MODEL, "effort": EFFORT, "cached": False, "tokens": tokens,
                           "seconds": round(time.time() - t0, 1),
                           "billed_to": "claude plan (subscription usage)" if self.plan == "subscription" else "API credits", **notes})
        return text
