# Contributing Guidelines

## Scope

This document defines coding, naming, and documentation conventions for firmware changes in this repository.

## Coding Style

- Use the project `.clang-format` style.
- Prefer explicit namespaces/types over file-scope `using namespace`.
- Keep functions focused and single-purpose where practical.
- Avoid hidden side effects in helper APIs; use clear names for stateful behavior.

## File Naming

- Use lowercase `snake_case` for firmware source/header filenames.
- Keep class/type names in `PascalCase`.
- Keep function/variable naming consistent with surrounding module style.

## Include Hygiene

- Include only what the file uses directly.
- In headers, prefer forward declarations when ownership/type size is not required.
- Remove stale includes when refactoring.

## Comments and Doxygen

- Use Doxygen file headers at top of each `.h/.cpp` file:
  - `@file`
  - `@brief`
  - optional `@details` when context is helpful
- Put API/function Doxygen comments in headers (not duplicated in `.cpp`) unless function is file-local.
- For non-trivial APIs, include `@param` and `@return`.
- Avoid placeholder comments like function-name-only briefs.

## Drivers

- Do not reformat or heavily rewrite vendor driver code under `src/drivers/` unless required for a bug fix.
- Keep local wrappers/helpers outside vendor files where possible.

## Validation

Before committing:

```bash
bash scripts/sanity_checks.sh
bash scripts/smoke_sensor_modes.sh
```

If PlatformIO is available, also run:

```bash
pio run
```

## Documentation Updates

- Update `README.md` and relevant files in `docs/` when behavior or structure changes.
- Update `REVISION_HISTORY.md` for release-relevant changes.
