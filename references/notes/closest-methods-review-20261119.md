# Closest methods: selected-section review

Reading level: methods and selected evaluation passages; full papers/appendices
not yet completely read. None reproduced. Date: 2026-11-19.

## ACE-CaP-0 — ace2026

Source: https://arxiv.org/html/2611.16872v1 (sections 2–3).
Policy development uses requested robot demonstrations and targeted simulated
experiments; deployment freezes code while still consuming runtime observations.
The guidance study has one development run per condition. Its reset allowance
excludes demonstration resets; reported evaluation repetitions must not be
mistaken for independent development runs. Implication: our comparisons must
include demonstration/reset costs and development variability.

Official project: https://acerobotics2025.github.io/ACE-CaP/.
GitHub API inspection of acerobotics2025/ACE-CaP on this date shows top-level
.gitignore, .nojekyll, index.html and static/. Repository license metadata is null.
This is presently evidence of a website, not an installable controller release.
Do not infer source-code reuse permission from paper availability. No claim that
code cannot exist elsewhere or will never be released.

## RoboRepair — schlesinger2024roborepair

Source: https://arxiv.org/html/2410.18893v1 (section 3).
Recovery generation consumes previous programs, execution traces and errors or
human interruptions. Prior state is available both as text and runtime data,
including visited places, held objects and responses. Thus recovery without
repeating completed actions is directly covered prior work. Its illustrated
service-robot API is substantially more abstract than continuous manipulation.
Open question: how it represents uncertain device effects, rather than known
failed actions. No absence claim until complete review.

Repository discovery has not located a confirmed author implementation. A name
search returns unrelated game-jam projects; none is accepted as paper code.

## Reliable CaP / NeSyRo — ahn2025reliablecap

Source: https://arxiv.org/html/2510.21302v1 (sections 3.1–3.4).
Uses a generated task specification, SMT verification, and sequential skill
validation combining language-model plausibility with PDDL feasibility. Low
confidence triggers recursive probe policies. Domain knowledge explicitly
includes skill preconditions/effects. Therefore applicability checks and active
observation are not new by themselves.

A third-party search result points to skku-ai/NESYRO, but GitHub cannot resolve
that repository; repository search returned no NeSyRo result. Treat the code
link as unverified, not official availability. Next inspect author/conference
sources and appendices. Do not reimplement and call it an official reproduction.
