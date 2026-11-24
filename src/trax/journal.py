"""Durable command intent and receipts; uncertainty never authorizes redispatch."""
from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from .contracts import canonical, digest


class Journal:
    def __init__(self, path: str | Path):
        self.db = sqlite3.connect(str(path), timeout=10)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, request TEXT NOT NULL, receipt TEXT)")
        self.db.execute("CREATE TABLE IF NOT EXISTS runs (id TEXT PRIMARY KEY, binding TEXT NOT NULL)")
        self.db.commit()

    def bind(self, run_id: str, binding: dict) -> None:
        value = digest(binding)
        with self.db:
            self.db.execute("INSERT OR IGNORE INTO runs VALUES (?, ?)", (run_id, value))
            old = self.db.execute("SELECT binding FROM runs WHERE id=?", (run_id,)).fetchone()[0]
            if old != value:
                raise ValueError("Run identity cannot be reused with different policy, parameters or deployment")

    def claim(self, command_id: str, request: dict) -> tuple[bool, dict | None]:
        value = canonical(request)
        with self.db:
            inserted = self.db.execute("INSERT OR IGNORE INTO commands VALUES (?, ?, NULL)", (command_id, value)).rowcount == 1
            old, receipt = self.db.execute("SELECT request, receipt FROM commands WHERE id=?", (command_id,)).fetchone()
            if old != value:
                raise ValueError("Command identity collision: action or arguments changed")
        # Transaction is committed before any physical dispatch.
        return inserted, json.loads(receipt) if receipt else None

    def record(self, command_id: str, receipt: dict) -> None:
        with self.db:
            self.db.execute("UPDATE commands SET receipt=? WHERE id=?", (canonical(receipt), command_id))

    def close(self):
        self.db.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
