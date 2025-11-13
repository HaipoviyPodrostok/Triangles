#!/usr/bin/env python3
import argparse
import difflib
import subprocess
import sys
from pathlib import Path
from typing import Iterable


# ---------- утилиты ----------

def color(s: str, c: str, *, enable: bool) -> str:
    if not enable:
        return s
    m = {
        "red": "\x1b[31m",
        "green": "\x1b[32m",
        "yellow": "\x1b[33m",
        "blue": "\x1b[34m",
        "dim": "\x1b[2m",
        "reset": "\x1b[0m",
    }
    return f"{m.get(c,'')}{s}{m['reset']}"


def normalize(t: str) -> str:
    """Убираем только CR, остальное сравниваем как есть."""
    return t.replace("\r", "")


def resolve_from_cwd(p: str | Path) -> Path:
    p = Path(p)
    return (Path.cwd() / p).resolve() if not p.is_absolute() else p.resolve()


def default_dirs(script_dir: Path) -> tuple[Path, Path]:
    tests = script_dir / "tests"
    keys = script_dir / "keys"
    return tests, keys


def iter_tests(test_dir: Path, pattern: str | None) -> Iterable[Path]:
    tests = sorted(p for p in test_dir.glob("*.txt"))
    if pattern:
        tests = [p for p in tests if pattern in p.name]
    return tests


# ---------- основная логика ----------

def run_one_test(
    bin_path: Path,
    test_file: Path,
    key_dir: Path,
    *,
    use_color: bool,
    max_diff_lines: int,
    update_keys: bool,
    show_pass: bool,
) -> bool:
    name = test_file.name
    expf = key_dir / name

    if not expf.exists():
        print(f"[{color('FAIL', 'red', enable=use_color)}] {name}  "
              f"{color(f'(missing expected: {expf})', 'dim', enable=use_color)}")
        return False

    # читаем вход
    data = test_file.read_bytes()

    try:
        proc = subprocess.run(
            [str(bin_path)],
            input=data,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
    except OSError as e:
        print(f"[{color('FAIL', 'red', enable=use_color)}] {name}")
        print("  " + color("Execution error:", "yellow", enable=use_color), e)
        return False

    status_ok = (proc.returncode == 0)
    stderr_ok = (len(proc.stderr) == 0)

    got_raw = proc.stdout.decode("utf-8", errors="replace")
    exp_raw = expf.read_text(encoding="utf-8", errors="ignore")

    got = normalize(got_raw)
    exp = normalize(exp_raw)

    stdout_ok = (got == exp)

    # автообновление ключей
    if update_keys and status_ok and stderr_ok and not stdout_ok:
        expf.write_text(got_raw, encoding="utf-8")
        exp = got
        stdout_ok = True

    ok = status_ok and stderr_ok and stdout_ok

    if ok:
        if show_pass:
            print(f"[{color('PASS', 'green', enable=use_color)}] {name}")
        return True

    # что-то пошло не так
    print(f"[{color('FAIL', 'red', enable=use_color)}] {name}")

    if not status_ok:
        print(f"  {color('Exit status:', 'yellow', enable=use_color)} {proc.returncode}")

    if not stderr_ok:
        err_lines = proc.stderr.decode(errors="replace").splitlines()
        head = "\n".join(err_lines[:5])
        print(f"  {color('Stderr (first 5 lines):', 'yellow', enable=use_color)}")
        print("    " + head.replace("\n", "\n    "))

    if not stdout_ok:
        diff = difflib.unified_diff(
            exp.splitlines(keepends=True),
            got.splitlines(keepends=True),
            fromfile=f"expected/{name}",
            tofile=f"actual/{name}",
            n=3,
        )
        print("  " + color("Diff:", "yellow", enable=use_color))
        for i, line in enumerate(diff):
            if i >= max_diff_lines:
                print("    ...")
                break
            sys.stdout.write("    " + line)

    return False


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    d_tests, d_keys = default_dirs(script_dir)

    ap = argparse.ArgumentParser(
        description="Run BIN < test.txt and compare stdout with key file."
    )
    ap.add_argument(
        "-b", "--bin",
        default=str(script_dir / "bin"),
        help=f"Path to executable (default: {script_dir/'bin'})",
    )
    ap.add_argument(
        "-t", "--test-dir",
        default=str(d_tests),
        help=f"Directory with input .txt (default: {d_tests})",
    )
    ap.add_argument(
        "-k", "--key-dir",
        default=str(d_keys),
        help=f"Directory with expected .txt (default: {d_keys})",
    )
    ap.add_argument(
        "-m", "--match",
        help="Run only tests whose filename contains this substring",
    )
    ap.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Do not print PASS tests, only FAIL",
    )
    ap.add_argument(
        "--no-color",
        action="store_true",
        help="Disable colored output",
    )
    ap.add_argument(
        "--max-diff-lines",
        type=int,
        default=80,
        help="Max number of diff lines to show per failed test (default: 80)",
    )
    ap.add_argument(
        "--stop-on-fail",
        action="store_true",
        help="Stop after first failed test",
    )
    ap.add_argument(
        "--update-keys",
        action="store_true",
        help="If binary output differs but exit code=0 and stderr empty, "
             "overwrite key file with actual output",
    )

    args = ap.parse_args()

    use_color = sys.stdout.isatty() and not args.no_color

    bin_path = resolve_from_cwd(args.bin)
    test_dir = resolve_from_cwd(args.test_dir)
    key_dir = resolve_from_cwd(args.key_dir)

    # проверки путей
    if not bin_path.exists():
        print(color("ERROR: --bin does not exist: ", "red", enable=use_color) + str(bin_path))
        sys.exit(2)
    if not bin_path.is_file():
        print(color("ERROR: --bin is not a file: ", "red", enable=use_color) + str(bin_path))
        sys.exit(2)
    if not test_dir.is_dir():
        print(color("ERROR: Test dir not found: ", "red", enable=use_color) + str(test_dir))
        sys.exit(2)
    if not key_dir.is_dir():
        print(color("ERROR: Key dir not found: ", "red", enable=use_color) + str(key_dir))
        sys.exit(2)

    tests = list(iter_tests(test_dir, args.match))
    if not tests:
        print(color(f"No test files in {test_dir}", "yellow", enable=use_color))
        sys.exit(2)

    print(color(f"BIN: {bin_path}", "dim", enable=use_color))
    print(color(f"TEST_DIR: {test_dir}", "dim", enable=use_color))
    print(color(f"KEY_DIR: {key_dir}", "dim", enable=use_color))
    if args.match:
        print(color(f"FILTER: '{args.match}'", "dim", enable=use_color))
    print()

    passed = failed = 0
    for tf in tests:
        ok = run_one_test(
            bin_path,
            tf,
            key_dir,
            use_color=use_color,
            max_diff_lines=args.max_diff_lines,
            update_keys=args.update_keys,
            show_pass=not args.quiet,
        )
        if ok:
            passed += 1
        else:
            failed += 1
            if args.stop_on_fail:
                break

    total = passed + failed
    print()
    print(
        f"Total: {total}  "
        f"{color('pass:', 'green', enable=use_color)} {passed}  "
        f"{color('fail:', 'red', enable=use_color)} {failed}"
    )

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
