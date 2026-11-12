# Contributing

Thank you for looking at Takt.

## Report a bug

Open an issue, anywhere in the repository, Puls included. Say what you did, what you expected and what happened.
For Puls, describe the behaviour in words: please do not post code for `sim/puls/` in an issue (see below).

## Pull requests

Pull requests are welcome for the website (`docs/`, `checks/website/`, `tools/website/`), the simulation (`sim/`
without `sim/puls/`) and the CAD (`cad/`, `hardware/`).

- **Inbound = outbound.** Your contribution is licensed under the licence of the folder or file it goes into (see
  [`LICENSE.md`](LICENSE.md)): MIT for code, CERN-OHL-S-2.0 for the CAD and part geometry, CC BY-SA 4.0 for texts and
  pictures, `MIT AND CC-BY-SA-4.0` for the mixed website files.
- **Sign off every commit** (`git commit -s`). The sign-off says you agree to the
  [Developer Certificate of Origin 1.1](https://developercertificate.org/): you wrote the change or have the right to
  submit it under that licence. This holds for code written with AI tools too.
- **Run the checks** that touch your change before you open the pull request. There is no CI yet: two of the checks
  measure speed and would fail on slow shared machines.

```bash
bun checks/website/guide-covers-bom.mjs docs
bun checks/website/meshes-roundtrip.mjs
node cad/viewer/selftest.mjs
sim/.venv/bin/python sim/checks/check_e1_all.py
sim/.venv/bin/python sim/checks/check_e2_all.py
```

## Puls (`sim/puls/`) takes no outside code

Puls is under AGPL-3.0-only. Code for `sim/puls/` from anyone else is not accepted: no pull requests, and no patches or
code snippets in issues or comments either, because such code cannot be used. This keeps the option to offer a
separate licence for Puls on request. Bug reports in words are welcome.

If you want to use Puls under other terms than the AGPL, ask in an issue titled "Puls licence".
