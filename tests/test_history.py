"""CLI regression tests for per-turn snapshots and retry/branch behavior."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "roadrunner.py"
GAME_NS = "http://arena.ai/roadrunner/game/v1"


class HistoryCLITests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.work = Path(self.tmp.name)
        self.game = self.work / "g.svg"
        self.history = self.work / "g.history"
        result = self.cli("new", str(self.game), "--style", "easy", "--seed", "11")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def tearDown(self):
        self.tmp.cleanup()

    def cli(self, *args):
        return subprocess.run(
            [sys.executable, str(SCRIPT), *map(str, args)],
            cwd=self.work, text=True, capture_output=True, check=False,
        )

    def run_turn(self):
        result = self.cli("run", str(self.game), "--steps", "1", "--fx", "0",
                          "--torque", "0", "--set", "weight=8", "--quiet")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def turn_files(self, directory=None):
        directory = directory or self.history
        return sorted(directory.glob("turn_*.svg"))

    def test_new_and_run_write_complete_snapshots_and_jsonl(self):
        self.assertEqual([p.name for p in self.turn_files()], ["turn_0000.svg"])
        self.assertTrue((self.history / "history.jsonl").is_file())

        self.run_turn()
        self.assertEqual([p.name for p in self.turn_files()],
                         ["turn_0000.svg", "turn_0001.svg"])
        root = ET.parse(self.history / "turn_0001.svg").getroot()
        self.assertIsNotNone(root.find(f"{{{GAME_NS}}}vehicle"))
        self.assertIsNotNone(root.find(f"{{{GAME_NS}}}path"))
        rows = [json.loads(line) for line in
                (self.history / "history.jsonl").read_text().splitlines()]
        self.assertEqual([row["turn"] for row in rows], [0, 1])
        self.assertEqual(rows[1]["event"], "run")
        self.assertEqual(rows[1]["applied"]["steps"], 1)
        self.assertEqual(rows[1]["applied"]["set"]["weight"], 8.0)
        self.assertIn("force_world", rows[1]["applied"])
        self.assertEqual(rows[1]["vehicle_after"]["weight"], 8.0)
        self.assertIn("coverage", rows[1])

    def test_retry_restores_exact_snapshot_and_reuses_turn_number(self):
        self.run_turn()
        self.run_turn()
        turn_one = (self.history / "turn_0001.svg").read_bytes()

        result = self.cli("retry", str(self.game), "--quiet")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.game.read_bytes(), turn_one)
        self.assertEqual([p.name for p in self.turn_files()],
                         ["turn_0000.svg", "turn_0001.svg"])

        self.run_turn()
        self.assertTrue((self.history / "turn_0002.svg").is_file())

    def test_keep_archives_repeated_attempts_without_overwriting(self):
        self.run_turn()
        self.run_turn()
        for attempt in range(2):
            result = self.cli("retry", str(self.game), "--keep", "--quiet")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.run_turn()

        branches = sorted(p for p in (self.history / "redo").iterdir() if p.is_dir())
        self.assertEqual(len(branches), 2)
        for branch in branches:
            self.assertTrue((branch / "turn_0002.svg").is_file())
            self.assertTrue((branch / "history.jsonl").is_file())
        result = self.cli("history", str(self.game))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("archived attempts", result.stdout)
        self.assertEqual(result.stdout.count("attempt_"), 2)

    def test_no_history_opt_out_clears_stale_turns(self):
        self.run_turn()
        result = self.cli("new", str(self.game), "--style", "easy", "--seed", "12",
                          "--force", "--no-history")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.turn_files(), [])
        self.assertFalse(self.history.exists())
        root = ET.parse(self.game).getroot()
        meta = root.find(f"{{{GAME_NS}}}meta")
        self.assertEqual(meta.get("history"), "off")
        result = self.cli("retry", str(self.game))
        self.assertEqual(result.returncode, 4)
        self.assertIn("--no-history", result.stderr)

    def test_legacy_game_starts_history_from_current_state(self):
        self.run_turn()
        # Simulate a pre-history or manually detached game: no sidecar folder.
        import shutil
        shutil.rmtree(self.history)
        self.run_turn()
        self.assertEqual([p.name for p in self.turn_files()],
                         ["turn_0000.svg", "turn_0001.svg"])
        rows = [json.loads(line) for line in
                (self.history / "history.jsonl").read_text().splitlines()]
        self.assertEqual(rows[0]["event"], "init")
        self.assertEqual(rows[1]["event"], "run")
        self.assertAlmostEqual(rows[0]["sim_time"], 1 / 60, places=2)

    def test_history_json_tail_zero_is_empty(self):
        self.run_turn()
        result = self.cli("history", str(self.game), "--json", "--tail", "0")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(result.stdout, "")


if __name__ == "__main__":
    unittest.main()
