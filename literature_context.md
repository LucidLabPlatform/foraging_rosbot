# Literature Search Context: Single Robot Foraging — Exploration vs. Exploitation

**Date:** 2026-03-28
**Purpose:** Context document for ongoing literature search. Use this to find additional relevant papers and frame the experimental contribution.

---

## 1. Project Setup

- **Platform:** Single ROSbot (physical robot)
- **Task:** Foraging — the robot navigates a bounded arena, detects coloured pucks using camera + depth sensor, and collects them
- **Arena:** Bounded physical arena with pucks distributed across it
- **Key ROS nodes:**
  - `1_perception_node.py` — colour-based puck detection
  - `2_depth_perception_node.py` — depth-filtered puck localisation
  - `RandomWalkServer` — random walk navigation strategy
- **Core research question:** How does the balance between exploration (random walk) and exploitation (directed navigation toward detected pucks) affect foraging performance?

---

## 2. What I Am Looking For

- Papers involving **robots performing a foraging task** in a **bounded arena**
- Where the robot (or robots) must **choose between exploration and exploitation**
- That **compare multiple algorithms or strategies** head-to-head
- That report **clear quantitative metrics** (e.g. items collected per time, collection rate, trial duration) that can serve as a **baseline for my own results**
- Preference for **single robot** setups, but multi-robot papers with single-robot conditions or per-robot metrics are acceptable
- Physical robot experiments preferred, but simulation is acceptable if metrics are well-defined

---

## 3. Key Terms and Concepts

| My Setup | Literature Equivalent |
|---|---|
| `RandomWalkServer` | Exploration phase — correlated random walk (CRW), Lévy walk, Brownian motion |
| Perception + directed navigation to puck | Exploitation phase — site fidelity, greedy assignment, informed search |
| Transition/balance between the two | Explore–exploit tradeoff parameter (α, ε-greedy, probability of switching) |
| Puck collection in bounded arena | Central-place foraging, object collection task |
| Home base return (if implemented) | Nest return, central-place foraging |

---

## 4. Papers Found — Ranked by Relevance

### 4.1 Best Practical Baseline (Physical Arena, Algorithm Comparison)

**"Comparing Physical and Simulated Performance of a Deterministic and a Bio-inspired Stochastic Foraging Strategy for Robot Swarms"**
- Authors: Nashed, Fricke, et al.
- Venue: IEEE IROS 2019
- Links: [IEEE](https://ieeexplore.ieee.org/document/8794240) | [UTRGV open copy](https://scholarworks.utrgv.edu/cs_fac/173/)

**Why relevant:** Compares two algorithms — CPFA (stochastic/exploration-biased) vs. DDSA (deterministic spiral/coverage-exploitation-biased) — in a physical arena with robots collecting physical objects. Results differ between simulation and physical, which is directly relevant to a physical robot study.

**Experiment summary:**
- Arena: Outdoor bounded ~100 m², objects (RFID tags / coloured discs) in clustered or uniform distributions
- Robots: iAnt ground robots, single-item carry capacity, return to central home base
- CPFA: Lévy-distributed random walk (explore) → site fidelity or pheromone recruitment (exploit), three evolved parameters control the tradeoff
- DDSA: Pre-computed spiral paths for guaranteed full coverage, deterministic, exploitation/coverage-focused
- Metrics: Number of tags collected over 20-minute fixed-duration trials, averaged over repeated runs
- Key finding: CPFA outperforms DDSA in physical conditions despite DDSA being better in simulation

---

### 4.2 Best Conceptual Framework

**"Towards an Engineering Science of Robot Foraging"**
- Authors: Winfield et al.
- Venue: Springer, 2009
- Links: [Springer](https://link.springer.com/chapter/10.1007/978-3-642-00644-9_16) | [ResearchGate PDF](https://www.researchgate.net/publication/265279374_Towards_an_Engineering_Science_of_Robot_Foraging)

**Why relevant:** Provides a formal taxonomy covering single and multi-robot foraging. Defines: search strategy (exploration), homing, collection efficiency (exploitation), and task allocation. Use to define terminology and justify metric choices. Explicitly covers single robots.

---

### 4.3 Best for Exploration Strategy Comparison

**"Random Walks in Swarm Robotics: An Experiment with Kilobots"**
- Authors: Dimidov, Oriolo, et al.
- Venue: ANTS 2016, Springer LNCS
- Links: [Semantic Scholar](https://www.semanticscholar.org/paper/Random-Walks-in-Swarm-Robotics:-An-Experiment-with-Dimidov-Oriolo/1e24e23bf658d24ff30d40acb479e8cfbe85a034) | [ResearchGate PDF](https://www.researchgate.net/publication/307144422_Random_Walks_in_Swarm_Robotics_An_Experiment_with_Kilobots)

**Why relevant:** Compares Brownian motion, correlated random walk (CRW), and Lévy walk head-to-head in a physical bounded arena. Although swarm, the individual robot behaviour is the unit of comparison. Provides direct metric comparisons (area covered per time, items reached) per walk type.

---

### 4.4 Best for Theoretical Exploration–Exploitation Framing

**"Explore vs. Exploit: Task Allocation for Multi-robot Foraging"**
- Authors: Baldassano & Leonard
- Venue: DARS 2009, Princeton
- Links: [PDF (Princeton)](https://naomi.princeton.edu/wp-content/uploads/sites/744/2021/03/BalLeo09.pdf) | [Semantic Scholar](https://www.semanticscholar.org/paper/Explore-vs-.-Exploit-:-Task-Allocation-for-Foraging-Baldassano-Leonard/87b65f273f65b3505f1854f6b1d987bf7c045f1e)

**Why relevant:** Cleanest formal framing of explore-vs-exploit in a foraging context. Defines explorer role (random walk, discovers new resources) vs. exploiter role (directed collection from known sites). Shows that optimal tradeoff is non-trivial — neither 0% nor 100% exploration is optimal. Metrics: total resources collected, collection rate, discovery latency. Though multi-robot, the tradeoff is defined at the individual level.

---

### 4.5 Best for Physical Robot + Large Scale Metrics

**"Sophisticated collective foraging with minimalist agents: a swarm robotics test"**
- Authors: Talamali, Bose, Haire, Xu, Marshall, Reina
- Venue: Swarm Intelligence, 2020
- Links: [Springer](https://link.springer.com/article/10.1007/s11721-019-00176-9) | [Open PDF (Sheffield Hallam)](https://shura.shu.ac.uk/25216/8/Talamali2019_Article_SophisticatedCollectiveForagin.pdf)

**Why relevant:** Validated on up to 200 physical robots with clear published metrics. Uses a single control parameter α that governs the explore/exploit tradeoff. Compares against Optimal Foraging Theory (OFT) predictions. Published numbers can be used as benchmark targets.

**Experiment summary:**
- Central nest surrounded by resource patches at varying distances/quality
- Agents use virtual pheromone trails (binary sensor) — pheromone attraction = exploit; random walk = explore
- Parameter α controls individual tendency to follow (exploit) vs. ignore (explore) trails
- Metrics: items collected per unit time, patch utilisation distribution, convergence time, OFT deviation
- Key finding: intermediate α values approximate OFT-optimal allocation; purely exploitative swarms converge on the nearest (not best) patch

---

### 4.6 Best for Metric Vocabulary

**"Balancing Collective Exploration and Exploitation in Multi-Agent and Multi-Robot Systems: A Review"**
- Authors: Frontiers in Robotics and AI, 2021
- Links: [Frontiers (open access)](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.771520/full) | [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC8844516/)

**Why relevant:** Systematically defines and compares metrics across the field. Use this to justify which metrics to measure and why. Covers: collection rate, exploration coverage, exploitation efficiency, adaptability to dynamic environments.

---

### 4.7 Comprehensive Review Paper

**"Swarm Foraging Review: Closing the Gap Between Proof and Practice"**
- Authors: Lu, Fricke, Ericksen & Moses
- Venue: Current Robotics Reports, 2020
- Links: [PDF](https://fricke.co.uk/Research/CurrentRoboticsReports2020.pdf) | [Springer](https://link.springer.com/article/10.1007/s43154-020-00018-1)

**Why relevant:** The single best survey of the field. Categorises approaches as self-organising vs. analytical. Explicitly discusses the reality gap (sim-to-real performance drops). Notes that augmented reality has enabled virtual pheromones with physical robots. Identifies that most foraging work remains in simulation — physical experiments are the gap. Use to frame contribution: LUCID bridges proof and practice by making physical experiments reproducible.

---

### 4.8 Price of Ignorance

**"Ignorance is Not Bliss: An Analysis of Central-Place Foraging Algorithms"**
- Authors: Fricke, Hecker, Griego, Tran & Moses
- Venue: IEEE/RSJ IROS 2019
- Links: [PDF](https://fricke.co.uk/Research/Price_Of_Ignorance_IROS2019.pdf)

**Why relevant:** Companion to Nashed et al. Defines the **"price of ignorance"** — quantifies performance loss from not knowing resource locations. Provides upper-bounds for expected complete collection times for CPFA variants (spiral-based, rotating-spoke, random-ballistic). The 3-mode FSM (explore→mixed→exploit) effectively reduces the price of ignorance as the robot discovers more.

---

### 4.9 Biological Mode Switching

**"Mode Switching in Organisms for Solving Explore-versus-Exploit Problems"**
- Authors: Biswas, Lamperski, Yang et al.
- Venue: Nature Machine Intelligence, 5, 1285–1296, 2023
- Links: [Nature MI](https://www.nature.com/articles/s42256-023-00745-y)

**Why relevant:** Studies discrete mode-switching in biological organisms (electric fish, humans, mice, moths). Key finding: organisms don't blend explore and exploit — they **discretely switch** based on sensory salience. Produces distinctive non-Gaussian velocity distributions. Directly validates the 3-mode FSM design over a continuous explore-exploit parameter.

---

### 4.10 Adaptive Energy Optimisation

**"Towards Energy Optimization: Emergent Task Allocation in a Swarm of Foraging Robots"**
- Authors: Liu & Winfield
- Venue: Adaptive Behavior, 15(3), 289–305, 2007
- Links: [SAGE](https://journals.sagepub.com/doi/10.1177/1059712307082088)

**Why relevant:** Three adaptation rules for dynamically adjusting forager-to-rester ratio using internal cues (successful retrieval), environmental cues (collisions), and social cues (teammate success). The internal-cue rule maps directly to Mode 0→1→2 transitions based on discovery thresholds (`explore_puck_pct`, `explore_corner_pct`).

Follow-up: **Liu & Winfield 2010** — "Modeling and Optimization of Adaptive Foraging in Swarm Robotic Systems" (IJRR) provides macroscopic probabilistic model. Physical e-puck experiments in ~2m×1.5m arena at Bristol Robotics Lab.
[SAGE](https://journals.sagepub.com/doi/abs/10.1177/0278364910375139)

---

### 4.11 Lévy-Modulated Correlated Random Walks in Bounded Arenas

**"Lévy-modulated Correlated Random Walks in Swarm Robotics"**
- Authors: (various)
- Venue: Swarm Intelligence, Springer, 2026
- Links: [Springer](https://link.springer.com/article/10.1007/s11721-026-00258-5)

**Why relevant:** Most recent and comprehensive study of random walk strategies in bounded spaces. Key finding: **a mild Correlated Random Walk outperforms Lévy walks in bounded arenas** due to frequent boundary collisions. Lévy walks preferred in open space. Optimal parameters: α ≈ 1.4, ρ ≥ 0.75 for minimising mean first passage time. Critical for the 3–4 m arena — `RandomWalkServer` should lean CRW.

---

### 4.12 Single-Robot Biomimetic Collection

**"Robot Collection and Transport of Objects: A Biomimetic Process"**
- Authors: Strombom & King
- Venue: Frontiers in Robotics and AI, vol. 5, article 48, 2018
- Links: [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2018.00048/full) | [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC7805832/)

**Why relevant:** Closest single-robot physical arena experiment to ours. Arena 880×435 mm, robot collects round objects via contact and delivers to collection zone. 4 trials with 2, 4, 8, and 16 objects. Finding: collection process **robust to initial object configuration** (low variance across trials). Validates varying puck counts across trials.

---

### 4.13 CPFA Foundational Paper

**"Beyond Pheromones: Evolving Error-Tolerant, Flexible, and Scalable Ant-Inspired Robot Swarms"**
- Authors: Hecker & Moses
- Venue: Swarm Intelligence, 9(1), 43–70, 2015
- Links: [Springer](https://link.springer.com/article/10.1007/s11721-015-0104-z)

**Why relevant:** Foundational CPFA paper. Describes full algorithm with GA optimisation. Evolved parameters (site fidelity rate, pheromone decay, recruitment probability) directly relevant to designing explore/exploit switching. More communication when resources clustered; less when clusters variable.

---

### 4.14 DDSA (Deterministic Baseline)

**"A Distributed Deterministic Spiral Search Algorithm for Swarms"**
- Authors: Fricke, Hecker, Griego, Tran & Moses
- Venue: IEEE/RSJ IROS 2016, pp. 4430–4436
- Links: [PDF](https://fricke.co.uk/Research/DDSA_FrickeIROS2016.pdf)

**Why relevant:** Guarantees complete arena coverage with minimal overlap. Linear relationship ~10.67 s/target (R²=0.998). 30-min experiments in 100 m² arena. Use as deterministic baseline against stochastic strategies.

---

### 4.15 Multiple-Place Foraging (Multiple Delivery Zones)

**"Multiple-Place Swarm Foraging with Dynamic Depots"**
- Authors: Lu, Hecker & Moses
- Venue: Autonomous Robots, 42, 2018
- Links: [Springer](https://link.springer.com/article/10.1007/s10514-017-9693-2)

**Why relevant:** Extends CPFA to **multiple collection zones (depots)** — directly analogous to our colour-matched corner delivery zones. Dynamic mobile depots improve collection efficiency by reducing return distance. Robots return to the *closest* depot.

---

### 4.16 NASA Swarmathon Competition

**"The Swarmathon: An Autonomous Swarm Robotics Competition"**
- Authors: Ackerman, Fricke, Hecker et al.
- Venue: arXiv:1805.08320, ICRA 2018 Workshop
- Links: [arXiv](https://arxiv.org/abs/1805.08320)

**Why relevant:** NASA competition — 3–6 Swarmie robots collect AprilTag-marked cubes from outdoor arenas (15–22 m²). Very similar task structure (find items, return to collection zone). Fully autonomous, no global map. CPFA-based baseline. 128–256 resources.

---

### 4.17 Information Flow and Explore/Exploit Balance

**"Information Flow Principles for Plasticity in Foraging Robot Swarms"**
- Authors: Pitonakova, Crowder & Bullock
- Venue: Swarm Intelligence, 10(1), 33–63, 2016
- Links: [Springer](https://link.springer.com/article/10.1007/s11721-016-0118-1)

**Why relevant:** Maximising information sharing good for static environments, but dynamic environments require regulated transfer to balance exploitation and exploration. Directly applicable to deciding when to switch modes.

---

### 4.18 Robot Object Sorting

**"Cache Consensus: Rapid Object Sorting by a Robotic Swarm"**
- Authors: Vardy, Vorobyev & Banzhaf
- Venue: Swarm Intelligence, 8(1), 61–87, 2014
- Links: [PDF](http://www.cs.mun.ca/~av/papers/si12_revised.pdf)

**Why relevant:** Sorting coloured pucks by type into homogeneous clusters — very close to our task. Each robot maintains a cache point per object type. Robots deposit pucks at the cache for that colour. Cache consensus enables implicit agreement on shared deposit locations.

---

### 4.19 Minimalist Object Clustering (Physical)

**"Clustering Objects with Robots That Do Not Compute"**
- Authors: Gauci, Chen, Li, Dodd & Gross
- Venue: AAMAS 2014, Paris
- Links: [ResearchGate](https://www.researchgate.net/publication/269576543_Clustering_Objects_with_Robots_That_Do_Not_Compute)

**Why relevant:** Physical e-puck experiment: 5 robots, 20 objects, 86.5% clustering in 10 minutes averaged over 10 trials. Provides a concrete physical baseline for object sorting metrics.

---

### 4.20 Foraging Metrics Survey

**"Multi-Agent Foraging: State-of-the-Art and Research Challenges"**
- Authors: Zedadra, Jouandeau, Seridi & Fortino
- Venue: Complex Adaptive Systems Modeling, 5(1), 2017
- Links: [Springer](https://link.springer.com/article/10.1186/s40294-016-0041-8)

**Why relevant:** Comprehensive survey cataloguing metrics across 50+ foraging papers. Most works use "total items collected" and "time to completion" as primary metrics. Proposes standardised comparison framework.

---

### 4.21 Mathematical Model of Foraging Interference

**"Mathematical Model of Foraging in a Group of Robots: Effect of Interference"**
- Authors: Lerman & Galstyan
- Venue: Autonomous Robots, 13, 127–141, 2002
- Links: [Springer](https://link.springer.com/article/10.1023/A:1019633424543)

**Why relevant:** Foundational mathematical model. Defines how interference affects performance. For a single-robot experiment, provides the interference-free baseline — maximum possible per-robot efficiency before multi-robot diminishing returns.

---

### 4.22 Reality Gap Dataset

**"On Using Simulation to Predict the Performance of Robot Swarms"**
- Authors: Ligot, Hasselmann et al.
- Venue: Scientific Data (Nature), 9, 788, 2022
- Links: [Nature](https://www.nature.com/articles/s41597-022-01895-1)

**Why relevant:** Dataset and methodology for comparing sim-vs-real. Physical robots show lower collection efficiency than simulation (expect 20–40% drop). Designing on more realistic simulation does NOT yield better physical performance. Physical testing remains essential.

---

### 4.23 Robotarium (Motion Capture Arena Reference)

**Georgia Tech Robotarium**
- Venue: IEEE RA Magazine, 2020
- Links: [Robotarium](https://www.robotarium.gatech.edu/)

**Why relevant:** 3.6 m × 4.3 m arena with Vicon sub-millimetre tracking at 120 Hz. Very similar scale to our 3–4 m arena with OptiTrack. Standard reference for motion-capture-based robot arena methodology.

---

### 4.24 FORDYCA Framework

**"Broadening Applicability of Swarm-Robotic Foraging Through Constraint Relaxation"**
- Authors: Harwell & Gini
- Venue: IEEE SIMPAR 2018
- Links: [PDF](https://www-users.cse.umn.edu/~gini/publications/papers/Harwell2018simpar.pdf) | [GitHub](https://github.com/swarm-robotics/fordyca)

**Why relevant:** Open-source FORDYCA framework (ARGoS + ROS). Measures four system properties: scalability, emergent self-organisation, flexibility, robustness. Also developed a predictive model using differential equations and random walk theory.

---

### 4.25 Task Partitioning in Foraging

**"Task Partitioning in Swarms of Robots: An Adaptive Method for Strategy Selection"**
- Authors: Pini, Brutschy, Frison, Roli, Dorigo & Birattari
- Venue: Swarm Intelligence, 5(3–4), 283–304, 2011
- Links: [Springer](https://link.springer.com/article/10.1007/s11721-011-0060-1)

**Why relevant:** Studies decomposing foraging into sub-tasks (search, pickup, transport, deposit). Adaptive task partitioning reduces physical interference, increases efficiency. Directly relevant to the phased approach (explore, mixed, exploit).

---

### 4.26 Autonomous Task Sequencing (Physical)

**"Autonomous Task Sequencing in a Robot Swarm"**
- Authors: Garattoni & Birattari
- Venue: Science Robotics, 3(20), eaat0430, 2018
- Links: [Science](https://www.science.org/doi/10.1126/scirobotics.aat0430)

**Why relevant:** 20 e-puck robots performing sequenced tasks in a physical arena. Planning emerges at the collective level. Demonstrates sequenced sub-task execution in physical hardware — relevant to validating the sequential mode structure.

---

### 4.27 Spatial AR for Physical Workspaces

**"ProjecTwin: A Digital Twin-Based Projection Framework for Flexible Spatial Augmented Reality"**
- Venue: Journal of Manufacturing Systems, 2024
- Links: [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0278612524002723)

**Why relevant:** Projecting digital-twin state onto a physical workspace. Concept matches our GAMA+TouchDesigner setup. Cite for the visualisation contribution.

---

### 4.28 Other Papers Encountered (Lower Priority)

| Paper | Key point | Link |
|---|---|---|
| MinDART — Rybski, Larson, Lindahl & Gini (JIRS 2008) | Physical multi-robot search & retrieval in bounded arena; simple random walk baseline | [PDF](https://www-users.cse.umn.edu/~gini/publications/papers/Rybsky07jirs.pdf) |
| Liemhetcharat et al. (Robotics 2015) | Proves delivery and foraging are formally equivalent | [MDPI](https://www.mdpi.com/2218-6581/4/3/365) |
| Pitonakova et al. (Frontiers 2018) | Information-Cost-Reward (ICR) framework for foraging | [Frontiers](https://www.frontiersin.org/articles/10.3389/frobt.2018.00047/full) |
| Lerman et al. (IJRR 2006) | Dynamic task allocation with stochastic observation-based switching | [SAGE](https://journals.sagepub.com/doi/10.1177/0278364906063426) |
| Ramachandran et al. (TRO 2020) | Information-correlated Lévy walk exploration and mapping | [arXiv](https://arxiv.org/abs/1903.04836) |
| Nauta et al. (ECAI 2020) | Collective Lévy walks with artificial potential fields | [PDF](https://ecai2020.eu/papers/486_paper.pdf) |
| Kegeleirs et al. (TAROS 2019) | Compares 5 random walk variants for mapping quality, 10 physical e-pucks | [Springer](https://link.springer.com/chapter/10.1007/978-3-030-25332-5_19) |
| Khaluf et al. (Applied Sciences 2019) | Scale-free features in foraging correlated with performance | [MDPI](https://www.mdpi.com/2076-3417/9/13/2667) |
| Kaminker et al. (Frontiers 2024) | Heterogeneous foraging swarms; coordination overhead vs. utility | [Frontiers](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2024.1426282/full) |
| A Comparative Analysis of Foraging Strategies using ARGoS (IEEE 2020) | Compares multiple foraging strategies in ARGoS simulator with foot-bots | [IEEE](https://ieeexplore.ieee.org/document/9202671) |
| Foraging with Forbidden Areas — Garzón Ramos (IRIDIA) | 30 e-pucks in diamond arena with forbidden zone; items delivered to nest | [Project](https://dgarzonramos.github.io/robotics101/srproject/) |
| GAMA Platform — MQTT + agent-based modelling | Built-in MQTT protocol support; tested with ActiveMQ; 5–10 connected simulations | [GAMA](https://gama-platform.org/wiki/UsingNetwork) |

---

## 5. Key Gap Identified

There is **no widely-cited paper** that does exactly:
- Single robot
- Bounded physical arena
- Object/puck collection with colour-matched delivery zones
- Adaptive explore→exploit mode switching
- Clear per-trial metrics
- Orchestrated by an IoT platform (experiment lifecycle, data collection, reproducibility)

This gap supports a **dual contribution**:
1. **Foraging:** Single-robot physical foraging with a 3-mode adaptive FSM in a bounded arena — filling the gap between swarm experiments (Nashed, Talamali) and simulation-only single-robot work.
2. **Systems:** LUCID as an experiment orchestration platform that makes physical robot experiments reproducible, observable, and operator-friendly — addressing the "proof-to-practice gap" identified by Lu et al. (2020).

Framing: "We adapt the foraging exploration–exploitation benchmark (Winfield 2009; Baldassano & Leonard 2009) to a single-robot physical setting with colour-sorted delivery, compare [algorithms], and demonstrate that an IoT orchestration layer (LUCID) materially improves experimental reproducibility and data richness — providing metrics directly comparable to [Nashed et al. 2019; Dimidov et al. 2016]."

---

## 6. Metrics to Consider Measuring

### 6.1 Standard Foraging Performance Metrics (from literature)

- **Items (pucks) collected per unit time** — primary throughput metric (universal across all surveyed papers)
- **Total items collected** over fixed-duration trial (e.g. 10 or 20 minutes)
- **Time to completion** — wall-clock time to place all pucks (Strombom 2018, Fricke 2016)
- **Time to first collection** — proxy for exploration efficiency
- **Fraction of time in exploration vs. exploitation mode** — characterises strategy balance
- **Coverage** — fraction of arena visited during exploration phase
- **Collection rate over time** — shows learning/adaptation curves if applicable
- **Path length** — integrate OptiTrack positions; path efficiency = distance per item (Harwell 2018)
- **Search time vs. transport time** breakdown (Lerman & Galstyan 2002)
- **Trial-to-trial variance** — measures consistency/reliability (Rybski 2008: communication reduced variance)

### 6.2 Practical / Operator-Facing Metrics (experiment usability)

These are not standard in the literature but are relevant for evaluating the practicality of a robotic system in real-world deployment. Use these as secondary metrics or in a discussion section comparing ease-of-use across algorithms.

- **Time to set up experiment** — total time from start to robot being ready to run (includes launching nodes, configuring parameters, placing pucks, calibrating sensors). Reflects deployment overhead of each algorithm.
- **Time to perform experiment** — total wall-clock duration of a single trial, including any required resets between runs. Reflects operator burden per data point collected.
- **Data gathered** — volume and richness of data produced per trial (e.g. number of topics logged, rosbag size, metrics automatically recorded vs. manually observed). Reflects how instrumented each approach is.
- **Number of commands to run experiment (complexity)** — number of terminal commands, launch files, or manual steps required to start a trial end-to-end. A proxy for reproducibility and operator error risk.
- **Experiment step durations** — from LUCID experiment_steps table (unique to orchestrated experiments)
- **Command latencies** — from LUCID commands table (system performance metric)

---

## 7. Experimental Design Recommendations (from literature)

| Parameter | Literature Standard | Recommendation |
|---|---|---|
| Trial duration | 10–20 min (Nashed: 20 min; biomimetic: until completion; DDSA: 30 min) | 15 min timeout (`trial_timeout_s: 900`) is well-aligned |
| Number of trials | 3–15 per condition (Nashed: multiple runs averaged; ANOVA studies: 3 reps; Gauci: 10 trials) | **Minimum 5 trials** per configuration, aim for 10 |
| Arena size | 880×435 mm (e-puck), 2×2 m (e-puck foraging), 3.6×4.3 m (Robotarium), ~100 m² (iAnt outdoor) | 3–4 m well-suited for ROSbot scale |
| Object count | 2–16 (biomimetic), 20 (Gauci), 128–256 (Swarmathon) | 6 pucks with 3 colours is reasonable for single robot |
| Statistical test | ANOVA + F-test (swarm), t-test (pairwise), non-parametric for small N | Use Wilcoxon rank-sum for small sample sizes |
| Random walk choice | CRW outperforms Lévy in bounded arenas (LMCRW 2026); α≈1.4 optimal | Lean toward CRW in `RandomWalkServer` |
| Reality gap | Expect 20–40% lower performance than simulation (Ligot 2022) | Physical testing essential; do not rely on simulation baselines |

---

## 8. Implementation Deviations from Theoretical Model (to note in paper)

### 8.1 Sector-Based Direction Sampling (not pure random walk)

The `RandomWalkServer` does **not** implement a pure uniform random walk for direction selection within a step's retry attempts. Instead, candidate directions are distributed evenly across the circle using a **sector-based sampling** approach:

- The circle is divided into `MAX_ORIENTATION_TRIES` equal sectors (currently 3 × 120°)
- A random phase offset `sector_offset ~ Uniform(0, sector_size)` is drawn once per step
- Attempt `k` uses direction `robot_yaw + sector_offset + k × sector_size`

**Reason for deviation:** Pure `Uniform(-π, π)` sampling per attempt caused repeated clustering of candidate directions across consecutive failed steps. Because `robot_yaw` is updated to the last attempted heading after each rotation, successive steps were biased toward similar directions. The sector approach guarantees within-step spread and, via the random phase, achieves a uniform distribution across many steps.

**Effect on algorithm:** The correlated random walk component (`walk_sigma > 0`, used for CPFA site fidelity returns) is **unchanged** — it still samples from `N(last_heading, walk_sigma)`. The sector approach only affects the uninformed exploration phase (`walk_sigma = 0`). In expectation over many steps, the direction distribution remains uniform, so the long-run statistical properties of the walk are preserved. The practical effect is better arena coverage with fewer wasted rotations.

**To note in paper:** The implemented random walk uses sector-based retry sampling rather than independent uniform sampling per attempt, in order to improve practical coverage on physical hardware. This is a minor implementation detail that does not alter the algorithmic identity of the CPFA exploration phase.

---

### 8.2 Goal Pre-validation via `make_plan` (filters invalid directions before rotation)

Before rotating toward a candidate direction, the robot pre-checks goal reachability using move_base's `/move_base/make_plan` service. Directions whose goals land inside wall inflation zones or obstacle regions are discarded without rotation.

**Effect on algorithm:** This biases the walk toward geometrically valid directions, effectively conditioning the random walk on the free-space distribution of the arena. In open arenas this has minimal effect; near walls it prevents the robot from attempting and failing directions toward obstacles. This is a navigation practicality constraint, not a CPFA design choice.

---

## 9. Next Steps

- [ ] Deep-read the 5 highest-priority new papers (Lu 2020 review, Fricke 2019 price-of-ignorance, Biswas 2023 mode-switching, LMCRW 2026, Strombom 2018 biomimetic)
- [ ] Determine how many conditions to test (e.g. different `explore_puck_pct` thresholds)
- [ ] Define the exact puck-colour-to-corner mapping for consistent trials
- [ ] Calibrate `RandomWalkServer` parameters against LMCRW 2026 recommendations
- [ ] Write the Related Work section using the taxonomy from Winfield 2009 + Zedadra 2017
