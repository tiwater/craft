# RoboRepair text review — 2026-11-19

Source: https://arxiv.org/html/2410.18893v1
Coverage: main text, appendices A/B and tables 1–7, including the floating
appendix tables omitted by an initial section-only extraction. Figure images
were not independently audited. Not reproduced; author implementation not
confirmed by the inspected primary links, which is not proof none exists.

Recovery uses prior execution state and trace context rather than restarting
completed work. This is direct prior art for generic recovery claims. The
benchmark covers API failures and human interruptions; one-error task coverage
does not establish repeated appliance-failure handling.

The evaluation varies prompts and completions. Tables separate task completion
from action overhead and contain unsuccessful tasks; overhead among successful
runs must not hide failures. The optimal-step reference has knowledge of the
future error and is not an attainable online policy baseline.

For cafe evaluation retain task completion, repeated physical actions, recovery
cost and unresolved outcomes separately. Coffee delivery in the paper does not
supply an oven or beverage-production capability contract. A local journal or
trace wrapper alone is insufficient differentiation for CRAFT.
