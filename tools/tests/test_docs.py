"""Documentation regressions using a small repository fixture."""
import unittest

from support import scratch_directory

import check_docs as DOCS
from common import write_text


class DocumentationTests(unittest.TestCase):
    def setUp(self):
        self.root = scratch_directory(self, "docs-test-")
        self.files = []

    def write(self, name, content):
        path = self.root / name
        write_text(path, content)
        self.files.append(path)
        return path

    def check(self):
        return DOCS.check(self.root, self.files)

    def test_links_fragments_reference_links_and_fenced_examples(self):
        self.write("README.md", "[Page](docs/page.md#clock--ownership)\n"
                   "[Repeated][again]\n[again]: docs/page.md#clock--ownership-1\n"
                   "[Web](https://example.invalid/not-a-local-file)\n"
                   "```md\n[Example](missing.md)\n```\n"
                   "~~~\n# Not a heading\n~~~\n"
                   "`[also an example](missing.md)`\n")
        self.write("docs/page.md", "# Clock & ownership\n# [Clock](../README.md) & ownership\n")
        self.assertEqual(self.check(), [])

    def test_missing_file_anchor_and_reference_are_distinct_errors(self):
        self.write("README.md", "[Missing](docs/missing.md)\n"
                   "[Heading](docs/page.md#absent)\n[Unknown][no-definition]\n")
        self.write("docs/page.md", "# Present\n")
        errors = self.check()
        self.assertEqual(len(errors), 3)
        self.assertIn("README.md:1: missing link target:", errors[0])
        self.assertIn("README.md:2: missing heading:", errors[1])
        self.assertIn("README.md:3: undefined link reference:", errors[2])

    def test_paths_targets_and_environment_names_with_generated_examples(self):
        self.write("CMakeLists.txt", "add_library(wpiemu_sdk INTERFACE)\n"
                   "option(WPIEMU_BUILD_APP \"Build\" ON)\n")
        self.write("sdk/source.cpp", 'auto value = getenv("WPIEMU_E2E");\n')
        self.write("README.md", "`sdk/source.cpp` `wpiemu_sdk` `WPIEMU_BUILD_APP` `WPIEMU_E2E`\n"
                   "`build/not-yet-configured` `plugins/<year>/source.cpp`\n")
        self.assertEqual(self.check(), [])
        self.write("docs/bad.md", "`sdk/removed.h` `wpiemu_removed` `WPIEMU_REMOVED`\n")
        self.write("sdk/comment.h", "// See docs/removed.md.\n")
        errors = self.check()
        self.assertEqual(len(errors), 4)
        self.assertTrue(any("missing repository path" in error for error in errors))
        self.assertTrue(any("unknown CMake target" in error for error in errors))
        self.assertTrue(any("unknown build/environment variable" in error for error in errors))
        self.assertTrue(any("missing documentation path" in error for error in errors))

    def test_adr_policy_does_not_ban_decision_index_or_internal_links(self):
        citation = "ADR " + "0001"
        self.write("docs/decisions/README.md", "[Boundary](0001-boundary.md)\n")
        self.write("docs/decisions/0001-boundary.md", f"# {citation}\n")
        self.write("README.md", "[Decisions](docs/decisions/README.md)\n")
        self.write("sdk/test_name.cpp", f'TEST_CASE("{citation} example") {{}}\n')
        self.write("sdk/macro.h", f'#define EXAMPLE "{citation}"\n')
        self.assertEqual(self.check(), [])
        self.write("sdk/comment.h", f"// See {citation}.\n")
        errors = self.check()
        self.assertEqual(len(errors), 1)
        self.assertIn("sdk/comment.h:1: use a current reference page", errors[0])

    def test_encoded_paths_duplicate_headings_and_repository_escape(self):
        self.write("docs/a page.md", "# A `clock`\n# A clock\n")
        self.write("README.md", "[Page](docs/a%20page.md#a-clock-1)\n"
                   "[Outside](../outside.md)\n")
        errors = self.check()
        self.assertEqual(len(errors), 1)
        self.assertIn("link leaves repository", errors[0])


if __name__ == "__main__":
    unittest.main()
