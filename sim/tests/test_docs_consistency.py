# SPDX-License-Identifier: MIT
"""The documentation hangs together: every link and every `docs/...` path in the repository points at a file that exists, every document is reachable from the index (docs/README.md), every
design, decision, verification and hardware document opens with a status line, and every "FILE.md section N" in the docs names a section that exists."""
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
SKIP_DIRS = {"build", ".git", "zephyr", "modules", "__pycache__", ".venv", "logs", ".west", "node_modules"}
TEXT_SUFFIXES = {".md", ".py", ".cpp", ".hpp", ".h", ".c", ".txt", ".sh", ".yml", ".yaml", ".overlay", ".conf"}


def doc_files():
    return sorted(p for p in DOCS.rglob("*.md"))


def repo_text_files():
    for p in ROOT.rglob("*"):
        if p.is_file() and not (set(p.relative_to(ROOT).parts) & SKIP_DIRS) and (p.suffix in TEXT_SUFFIXES or p.name == "Kconfig"):
            yield p


def headings(path: Path) -> set[str]:
    """The section numbers a document defines: '## 3. Title' and '### 3.2 Title' give '3' and '3.2'; '## 9b. Title' gives '9b'."""
    out = set()
    for line in path.read_text().splitlines():
        m = re.match(r"^#{2,4} (\d+[a-z]?(?:\.\d+)*)[.:\s]", line)
        if m:
            out.add(m.group(1))
    return out


class Links(unittest.TestCase):
    def test_every_relative_markdown_link_in_the_docs_resolves(self):
        bad = []
        for p in doc_files() + [ROOT / "README.md", ROOT / "CONTRIBUTING.md"]:
            for m in re.finditer(r"\]\(([^)#\s]+)(#[^)]*)?\)", p.read_text()):
                target = m.group(1)
                if re.match(r"[a-z]+://|mailto:", target):
                    continue
                if not (p.parent / target).resolve().exists():
                    bad.append(f"{p.relative_to(ROOT)} -> {target}")
        self.assertEqual(bad, [])

    def test_every_docs_path_mentioned_anywhere_in_the_repository_exists(self):
        bad = []
        for p in repo_text_files():
            if p.name == "test_docs_consistency.py":
                continue
            for m in re.finditer(r"docs/((?:[A-Za-z_]+/)*[A-Za-z0-9_-]+\.md)", p.read_text()):
                if not (DOCS / m.group(1)).exists():
                    bad.append(f"{p.relative_to(ROOT)}: docs/{m.group(1)}")
        self.assertEqual(sorted(set(bad)), [])


class Index(unittest.TestCase):
    def test_every_document_is_listed_in_its_index(self):
        top = (DOCS / "README.md").read_text()
        procedures = (DOCS / "procedures" / "README.md").read_text()
        missing = []
        for p in doc_files():
            rel = str(p.relative_to(DOCS))
            if p.name == "README.md":
                continue
            if rel.startswith("procedures/"):
                if p.name not in procedures:
                    missing.append(f"{rel} (docs/procedures/README.md)")
            elif rel not in top:
                missing.append(f"{rel} (docs/README.md)")
        self.assertEqual(missing, [], "add these to their index")

    def test_the_indexes_list_nothing_that_does_not_exist(self):
        bad = []
        for index, base in ((DOCS / "README.md", DOCS), (DOCS / "procedures" / "README.md", DOCS / "procedures")):
            for m in re.finditer(r"\]\(([^)#\s]+\.md)", index.read_text()):
                if not (base / m.group(1)).resolve().exists():
                    bad.append(f"{index.relative_to(ROOT)} -> {m.group(1)}")
        self.assertEqual(bad, [])


class Headers(unittest.TestCase):
    def test_every_design_decision_verification_and_hardware_document_opens_with_a_status_line(self):
        bad = []
        for sub in ("design", "decisions", "verification", "hardware", "project"):
            for p in sorted((DOCS / sub).glob("*.md")):
                head = p.read_text().split("\n", 6)[:6]
                if not any(line.startswith("> Status:") for line in head):
                    bad.append(str(p.relative_to(DOCS)))
        self.assertEqual(bad, [])

    def test_the_status_line_uses_one_of_the_agreed_words(self):
        words = ("built", "designed", "accepted", "reference", "plan", "procedure", "in force", "decided", "proposed", "template", "written", "catalogue", "measured", "register")
        bad = []
        for sub in ("design", "decisions", "verification", "hardware", "project"):
            for p in sorted((DOCS / sub).glob("*.md")):
                line = next((l for l in p.read_text().split("\n", 6)[:6] if l.startswith("> Status:")), "")
                if not any(w in line.lower() for w in words):
                    bad.append(f"{p.relative_to(DOCS)}: {line[:80]}")
        self.assertEqual(bad, [])


class Sections(unittest.TestCase):
    def test_every_file_section_reference_names_a_section_that_exists(self):
        by_name = {p.name: p for p in doc_files()}
        bad = []
        pattern = re.compile(r"`?(?:docs/(?:[a-z]+/)?)?([A-Z][A-Z_]+\.md)`?,? (?:section|sections) (\d+[a-z]?(?:\.\d+)*)")
        for p in list(repo_text_files()):
            if p.name == "test_docs_consistency.py":
                continue
            for m in pattern.finditer(p.read_text()):
                name, sec = m.groups()
                if name in by_name and sec not in headings(by_name[name]):
                    bad.append(f"{p.relative_to(ROOT)}: {name} section {sec}")
        self.assertEqual(sorted(set(bad)), [])


class Numbers(unittest.TestCase):
    """The headline numbers are quoted in several pages (README, STATUS, PROOF, the standard, the register); they must not drift apart from their source."""

    def test_the_campaign_total_quoted_anywhere_is_the_one_in_the_generated_table(self):
        text = (DOCS / "verification" / "FAULT_CAMPAIGN.md").read_text()
        m = re.search(r"\| \*\*total\*\* \| \*\*([\d,]+)\*\* \| \*\*([\d,]+)\*\* \|", text)
        self.assertIsNotNone(m, "the generated total row is missing from FAULT_CAMPAIGN.md")
        scenarios = m.group(1)
        bad = []
        for p in doc_files() + [ROOT / "README.md"]:
            for q in re.finditer(r"(\d{1,3}(?:,\d{3})+) (?:fault )?scenarios", p.read_text()):
                if q.group(1) != scenarios:
                    bad.append(f"{p.relative_to(ROOT)}: {q.group(0)} (the table says {scenarios})")
        self.assertEqual(bad, [])

    def test_the_mutant_count_quoted_anywhere_is_the_number_of_mutants(self):
        sys.path.insert(0, str(ROOT / "tools" / "mutation"))
        import mutations

        n = len(mutations.MUTATIONS)
        bad = []
        for p in doc_files() + [ROOT / "README.md", ROOT / "sim" / "README.md"]:
            for q in re.finditer(r"(\d{3}) (?:mutants|injected bugs|deliberate bugs)", p.read_text()):
                if int(q.group(1)) != n:
                    bad.append(f"{p.relative_to(ROOT)}: {q.group(0)} (there are {n})")
        self.assertEqual(bad, [])

    def test_the_simulator_mutant_count_quoted_anywhere_is_the_number_of_simulator_mutants(self):
        sys.path.insert(0, str(ROOT / "tools" / "mutation"))
        import sim_mutations

        n = len(sim_mutations.MUTATIONS)
        bad = []
        for p in doc_files() + [ROOT / "README.md", ROOT / "sim" / "README.md", ROOT / "CONTRIBUTING.md"]:
            for q in re.finditer(r"(\d+) simulator mutants", p.read_text()):
                if int(q.group(1)) != n:
                    bad.append(f"{p.relative_to(ROOT)}: {q.group(0)} (there are {n})")
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
