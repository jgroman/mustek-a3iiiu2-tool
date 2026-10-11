"""Allow running as `python -m mustek_driver` (CLI) or `python -m mustek_driver gui`."""
import sys

if len(sys.argv) > 1 and sys.argv[1] == "gui":
    from mustek_driver.gui import run
    run()
else:
    from mustek_driver.cli import main
    sys.exit(main())
