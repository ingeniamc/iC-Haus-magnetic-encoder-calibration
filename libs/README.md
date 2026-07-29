# libs/

This folder holds local wheel (`.whl`) dependencies that are not available on
public PyPI.

## Required files

| File | Version | Used by |
|------|---------|---------|
| `mu_3sl-3.4.3.1.post2-py3-none-any.whl` | 3.4.3.1 | `poetry install` (via `pyproject.toml`) |

## How to

Place the `.whl` file in this directory before running `poetry install`.

## Notes

- The `.whl` files, and any other file, on this folder are **not** version-controlled (they are git-ignored).
- Only this README is tracked to ensure the folder structure exists in the repo.
