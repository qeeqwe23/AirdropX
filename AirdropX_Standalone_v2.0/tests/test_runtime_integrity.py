import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

class RuntimeIntegrityTests(unittest.TestCase):
    def test_runtime_does_not_launch_matlab(self):
        bad=('matlab.exe','-batch','matlab.engine','subprocess.')
        for p in (ROOT/'core').glob('*.py'):
            txt=p.read_text(encoding='utf-8').lower()
            for token in bad: self.assertNotIn(token.lower(),txt,f'{token} in {p.name}')
    def test_python_sources_are_clean_utf8(self):
        for folder in ('core','ui','tools','tests'):
            for p in (ROOT/folder).glob('*.py'):
                txt=p.read_text(encoding='utf-8')
                self.assertNotIn('\ufffd',txt,p.name)
                self.assertNotIn(chr(0x951f),txt,p.name)
