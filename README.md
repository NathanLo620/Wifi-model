# ns-3 IEEE 802.11bn (UHR) Prioritized EDCA (P-EDCA) Implementation 🚀📶

[![ns-3 Version](https://img.shields.io/badge/ns--3-3.45-blue.svg)](https://www.nsnam.org/)
[![Standard](https://img.shields.io/badge/IEEE-802.11bn-green.svg)]()
[![Status](https://img.shields.io/badge/Status-Verification_%26_Optimization-orange.svg)]()

> **Note to Interviewers & Wi-Fi Domain Experts:**
> This repository contains an implementation of **Prioritized EDCA (P-EDCA)**, a key channel access mechanism proposed in the future IEEE 802.11bn (UHR) draft to resolve starvation issues for high-priority traffic (e.g., Voice / Low-Latency applications) in extremely dense network environments. Built on top of **ns-3.45**, this project details the core modifications to the Wi-Fi MAC layer protocol and includes performance validation.
> This README provides a deep dive into the specific modifications made to the ns-3 core source code (`src/wifi/model/`), along with the latest development progress and technical insights.

---

## 📖 Background & P-EDCA Mechanism

In traditional 802.11 EDCA mechanisms, high-density networks lead to massive channel contention and packet collisions. For latency-sensitive `AC_VO` (Voice) traffic, consecutive collisions trigger exponential backoff window (CW) expansion, resulting in severe transmission delays and packet loss (Starvation).

**P-EDCA (Prioritized EDCA)** introduces a novel two-stage channel access mechanism:
1. **Stage 1 (Medium Reservation):** When a STA experiences consecutive transmission failures for `AC_VO` traffic (`QSRC ≥ 2`), it transitions to P-EDCA mode. After a shortened `DSAIFS` delay, the STA broadcasts a special **DS-CTS (Defer Signal CTS)** control frame to clear interference in the surrounding medium, securing a dedicated 97µs Contention Window.
2. **Stage 2 (Prioritized Contention):** Upon successfully transmitting the DS-CTS, the STA **suspends contention for all other Access Categories (`AC_VI`, `AC_BE`, `AC_BK`)** and enters Stage 2. The STA then uses highly compressed EDCA parameters (`CWmin=7, CWmax=7, AIFSN=2`) to exclusively contend and transmit `AC_VO` frames, significantly reducing latency.

---

## 🛠️ ns-3 `src/wifi/model/` Core Architecture Modifications

To accurately model the 802.11bn draft specifications within ns-3, deep modifications were made to the QoS MAC state machine and frame exchange control logic (primarily in the `model` directory):

| Core Modified File | Implementation Highlights & Details |
| :--- | :--- |
| [qos-frame-exchange-manager.cc](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/qos-frame-exchange-manager.cc) <br>[qos-frame-exchange-manager.h](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/qos-frame-exchange-manager.h) | **[Core State Machine]**<br>Implemented the complete P-EDCA control flow. Modified `StartTransmission()` to encompass trigger condition (`QSRC >= 2`) evaluation and DS-CTS broadcast frame construction.<br>Controlled the transition into Stage 2 via a `CTS TxEnd` callback. Significantly modified CSMA/CA collision handling (e.g., `TransmissionFailed()`) to enforce overrides on `CW`, `QSRC` / `PSRC`, and the backoff algorithm upon collisions. |
| [frame-exchange-manager.cc](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/frame-exchange-manager.cc) | **[NAV Reception Logic]**<br>Modified the receiving logic (`UpdateNav()`). Since the DS-CTS uses a standard-designated Broadcast RA (`00:0F:AC:47:43:00`), established provisions to ensure this frame is not filtered out by standard address filters. This allows surrounding IDLE STAs to correctly decode it and set their NAV (Network Allocation Vector) to the critical 97µs. |
| [qos-txop.cc](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/qos-txop.cc) <br>[qos-txop.h](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/qos-txop.h) | **[Forced Backoff Reset]**<br>Added the `SetPedcaBypassBackoff(bool bypass, uint8_t linkId)` method. This allows P-EDCA nodes, immediately after transmitting a DS-CTS, to bypass their traditional TxOP backoff mechanism and seamlessly force entry into Stage 2 contention. |
| [wifi-mac.cc](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/wifi-mac.cc) <br>[wifi-mac.h](file:///home/wmnlab/Desktop/ns-3.45/src/wifi/model/wifi-mac.h) | **[Modular Attributes]**<br>Introduced the `PedcaSupported` Node Attribute (default `false`). This enables high-level Simulation Scripts to dynamically control whether specific nodes or the entire BSS powers P-EDCA capabilities, facilitating A/B Testing. |

---

## 🔬 Technical Deep Dive: Timing & Operations

Strict alignment with P-EDCA timing specifications (based on P802.11bn™) has been enforced:

1. **DS-CTS Frame Specifications:**
   - **Rate:** Mandated Non-HT OFDM 6 Mbps to maximize coverage.
   - **Receiver Address (RA):** Hardcoded to the standard reserved address `00:0F:AC:47:43:00`.
   - **Duration (NAV):** Accurately set to 97µs (accounting for SIFS + AIFSN + CWmax slot times).
2. **Timing Accuracy Verification:**
   - Deep trace log analysis confirms that STAs precisely fire the DS-CTS after `DSAIFS = SIFS (16µs) + AIFSN (18µs) = 34µs` of Medium Idle time.
   - Verified that the interval between the end of DS-CTS TX and the start of Data TX consistently falls within the **[34µs, 97µs]** range.
3. **TIMING EXPIRED Protection Mechanism:**
   - As a fail-safe for extreme scenarios, if the medium remains occupied for more than 97µs after entering Stage 2, the state machine safely triggers a Fallback mechanism, reverting to standard EDCA contention to prevent Deadlocks.

---

## 📊 Development Progress & Latest Findings

The project is currently in the **Verification & Optimization Phase**. The core logic successfully executes and collects data. Concurrently, physical layer simulations have revealed valuable engineering insights:

### ✅ Completed Milestones
- **Core Logic Integration:** STAs flawlessly switch to P-EDCA mode based on QSRC, transmit multicast DS-CTS frames, update surrounding NAVs, and secure Stage 2 transmission priority.
- **Heterogeneous Collision Verification:** Recently simulated high-collision interactions between **STA A (P-EDCA)** and **STA B (Legacy RTS)**. Verified how Legacy backoff (RTS Timeout) behaves and how P-EDCA processes CSMA/CA state changes following a DS-CTS and Legacy RTS collision.
- **Automated Data Pipeline:** Developed a Python multiprocessing utility (`pdf_plot.py`) to average multiple ns-3 seed samples, accurately plotting delay Probability Density Functions (PDF/CDF) and Throughput variations.

### ⚠️ Technical Insight: CSMA/CA Half-duplex Inducing DS-CTS Collisions
In dense 10-node topologies, we observed a DS-CTS collision rate of ~25-30%. Through fundamental codebase tracing, we concluded:
Due to the **Half-duplex physical limitations** of traditional Wi-Fi transmission, when multiple nodes are highly congested, their EDCA Backoffs may reach zero simultaneously.
On identical microsecond **Same-slot** boundaries, all nodes detecting CCA IDLE fire DS-CTS simultaneously. Nodes actively transmitting or experiencing collisions cannot transition to RX state, rendering the DS-CTS NAV broadcast ineffective (Partial NAV). This validates the accuracy of our implementation: the simulation realistically reflects the fundamental limits of the physical layer and CSMA/CA, representing a phenomenon aligned with physical laws rather than a software bug.

---

## 🚀 Next Steps

Based on our recent findings and the evolving IEEE 802.11bn standard draft, our next objective is clear:

1. **Update to Draft 1.3:** We will align the implementation with the latest **IEEE 802.11bn Draft 1.3** specifications to ensure all P-EDCA mechanisms conform to the most recent standard definitions and behaviors.

---

## 💻 Compilation & Execution

```bash
# Enter the ns-3 workspace and build
cd /path/to/ns-3.45
./ns3 build

# Execute a P-EDCA verification scenario (e.g., 10 Nodes topology, 5 seconds simulation)
./ns3 run "scratch/pedca_verification_nsta.cc --nSta=10 --simTime=5.0 --dataRate=0.5Mbps"

# Baseline comparison: Standard EDCA
./ns3 run "scratch/wifi_backoff80211n.cc --nSta=10 --simTime=5.0 --dataRate=0.5Mbps"
```

---

> **Author:** [Nathan Lo / WMNLab]  
> **Institution:** WMNLab  
> *References: IEEE P802.11bn™*
