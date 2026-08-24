#!/usr/bin/env python3
"""BPI 板型資訊初始化順序的來源碼回歸測試。"""

from pathlib import Path
import unittest


REPO_DIR = Path(__file__).resolve().parents[1]
SOURCE = REPO_DIR / "wiringPi/wiringPi_bpi.c"


class BpiBoardIdTests(unittest.TestCase):
    def test_board_id_uses_cached_bpi_layout(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        function = source.split("void bpi_piBoardId", maxsplit=1)[1].split(
            "int bpi_wiringPiSetup", maxsplit=1
        )[0]
        self.assertIn("gpioLayout = bpi_piGpioLayout ()", function)
        self.assertNotIn("gpioLayout = piGpioLayout ()", function)


if __name__ == "__main__":
    unittest.main()
