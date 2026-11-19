# CaP literature registry

Scope: code policies, execution feedback, repair, skill reuse, verification and
relevant evaluation. **Scene reconstruction is excluded.** This is a maintained
research map, not an exhaustive systematic review. Snapshot: 2026-11-19.

- [references.bib](references.bib): 22 citeable arXiv entries; authors and titles
  retrieved from primary-page citation metadata. Publication year is the arXiv
  submission year, not necessarily the conference year. No invented venues/DOIs.
- [catalog.json](catalog.json): stable keys, original metadata, reading status,
  relevance, provenance and follow-up questions.
- [novelty-matrix.md](novelty-matrix.md): claims already covered and required comparisons.
- [reading-plan.md](reading-plan.md): paper-reading and reproduction priorities.

**Reading depth:** initial abstract/project-page review; selected method sections
for ACE-CaP-0, RoboRepair and NeSyRo now have linked notes in catalog.json. No entry is marked
full-text reviewed or reproduced. Repository availability does not mean we have
run its code, audited its license or checked its reported scores. Bibliographic
verification does not verify experimental claims. These distinctions must remain
visible when preparing a manuscript.

## Inventory

| Cite key | Work (first preprint year) | Why retain it |
| --- | --- | --- |
| `liang2022cap` | [Code as Policies: Language Model Programs for Embodied Control](https://arxiv.org/abs/2209.07753) (2022) | 自然语言生成调用感知/控制 API 的策略程序；基础对照。 |
| `singh2022progprompt` | [ProgPrompt: Generating Situated Robot Task Plans using Large Language Models](https://arxiv.org/abs/2209.11302) (2022) | 程序化接口、前置条件与恢复已有先例。 |
| `huang2023voxposer` | [VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models](https://arxiv.org/abs/2307.05973) (2023) | 生成代码可缓存并随观测闭环执行；不能宣称策略缓存是新贡献。 |
| `huang2024rekep` | [ReKep: Spatio-Temporal Reasoning of Relational Keypoint Constraints for Robotic Manipulation](https://arxiv.org/abs/2409.01652) (2024) | 目标和路径约束程序；并非通用技能记忆系统。 |
| `chen2024roboscript` | [RoboScript: Code Generation for Free-Form Manipulation Tasks across Real and Simulation](https://arxiv.org/abs/2402.14623) (2024) | 统一仿真与真实接口、代码部署；与 2026 同名项目区分。 |
| `mu2024robocodex` | [RoboCodeX: Multimodal Code Generation for Robotic Behavior Synthesis](https://arxiv.org/abs/2402.16117) (2024) | 多模态树形任务分解及模型训练。 |
| `hu2023codebotler` | [Deploying and Evaluating LLMs to Program Service Mobile Robots](https://arxiv.org/abs/2311.11183) (2023) | CodeBotler/RoboEval；用时序逻辑验证多初始状态下执行轨迹。 |
| `zha2023droc` | [Distilling and Retrieving Generalizable Knowledge for Robot Manipulation via Language Corrections](https://arxiv.org/abs/2311.10678) (2023) | 从人的语言纠正中蒸馏并检索经验；不等同全自动修复。 |
| `zhu2024incoro` | [InCoRo: In-Context Learning for Robotics Control with Feedback Loops](https://arxiv.org/abs/2402.05188) (2024) | 动态环境中持续反馈控制，与冻结代码部署区分。 |
| `schlesinger2024roborepair` | [Creating and Repairing Robot Programs in Open-World Domains](https://arxiv.org/abs/2410.18893) (2024) | 从当前状态恢复并减少重复动作；直接相关工作。 |
| `ahn2025reliablecap` | [Towards Reliable Code-as-Policies: A Neuro-Symbolic Framework for Embodied Task Planning](https://arxiv.org/abs/2510.21302) (2025) | 符号验证和主动获取缺失观测；证据条件选题需直接比较。 |
| `lu2026aspire` | [ASPIRE: Agentic /Skills Discovery for Robotics](https://arxiv.org/abs/2607.00272) (2026) | 多模态执行记录、程序修复、技能库与搜索；核心对照。 |
| `enpire2026` | [ENPIRE: Agentic Robot Policy Self-Improvement in the Real World](https://arxiv.org/abs/2606.19980) (2026) | 自动复位、执行、验证与改进；包括非 CaP 的学习路线。 |
| `xie2026roboskill` | [Explore, Execute, Evolve: A Skill Acquisition and Reuse Loop for Embodied Agents](https://arxiv.org/abs/2609.37810) (2026) | 文本和代码技能复用及成本摊销；核心对照。 |
| `fu2026capx` | [CaP-X: A Framework for Benchmarking and Improving Coding Agents for Robot Manipulation](https://arxiv.org/abs/2603.22435) (2026) | CaP-Gym/Bench/Agent0/RL；控制 API 抽象和反馈条件。 |
| `chen2026amp` | [On Agentic Manipulation Policies](https://arxiv.org/abs/2611.09892) (2026) | AMP1 实时智能体执行和记忆；与冻结策略分别比较成本。 |
| `ace2026` | [ACE-CaP-0: Real-World Code-as-Policies in the Era of Experience](https://arxiv.org/abs/2611.16872) (2026) | 按需示范/仿真，部署期冻结程序；核心对照。 |
| `fan2026captain` | [CaPtain: A Pipeline for Post-Training Code-as-Policy Robot Agents](https://arxiv.org/abs/2610.40799) (2026) | 小模型的 SFT/RL 后训练；不属于纯框架改进。 |
| `roboscript2026` | [World In-Context Learning for Agentic Robot Manipulation](https://arxiv.org/abs/2611.12885) (2026) | 示范、任务记忆、冻结部署；必须与 2024 同名工作区分。 |
| `wang2023voyager` | [Voyager: An Open-Ended Embodied Agent with Large Language Models](https://arxiv.org/abs/2305.16291) (2023) | Minecraft 中代码技能库和自动课程；不能视为真实机器人证据。 |
| `ha2023scalingup` | [Scaling Up and Distilling Down: Language-Guided Robot Skill Acquisition](https://arxiv.org/abs/2307.14535) (2023) | 代码引导采集/验证后蒸馏为神经策略；与部署代码策略区分。 |
| `li2026roboclaw` | [RoboClaw: An Agentic Framework for Scalable Long-Horizon Robotic Tasks](https://arxiv.org/abs/2603.11558) (2026) | 数据采集、策略学习、执行及复位；并非纯 CaP。 |

## Manuscript use

Use stable keys (for example `\cite{lu2026aspire,ace2026}`) from references.bib.
Before submission, check the latest official version, author list and published
venue, and record the exact version supporting each claim. Prefer BibLaTeX/Biber
for Unicode author names. Add page/section references to the catalog after full
reading; do not cite an abstract as proof of a detailed mechanism or limitation.

When a name is ambiguous, preserve title + identifier: RoboScript (2024) and the
2026 project are different works. Voyager is Minecraft, not physical-robot
validation. ENPIRE/RoboClaw/scaling-up include learned-policy workflows and are
not interchangeable with frozen Python policies. Newer papers remain preprints
unless an official publication is confirmed.

## Maintenance rule

Every newly discussed project must receive a catalog entry or an explicit
out-of-scope note. Preserve older entries; supersession changes the status, not
history. Store source links, not unlicensed full-paper copies. Keep unresolved
questions explicit. Update catalog and BibTeX together; tests check key coverage.
