# CRCM
 Only the code for the key module CRCM is available at this stage of the study, and the complete project code will be considered for addition after the paper is published.
# A Following Collaborative Robot Harvesting-Assisted Transport System with Collaborative Region Constraints Model  

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Introduction
The mechanization of fruit harvesting is essential for sustainable development in modern fruit industries. However, current automation technologies still face challenges in efficiency, cost, and adaptability to unstructured environments. This paper proposes a Following Collaborative Robot Harvesting-Assisted Transport System (FCR-HATS) integrated with Collaborative Region Constraints Model (CRCM), which aims to improve the efficiency of agricultural harvesting and reduce the intensity of manual labour using human-robot collaboration. CRCM introduces a decision-planning mechanism, transforming conventional following behaviour into three execution states: following, stopping, and monitoring, thereby enabling a shift from passive following to proactive cooperation. Field experiments demonstrate that CRCM significantly improves trajectory overlap accuracy, reducing the average AMC by 0.19 compared to traditional models, while mitigating collision risks without requiring additional obstacle avoidance strategies. With a unit shrinkage distance of 0.05m, the FCR achieves precise stopping with lateral and longitudinal errors of 0.17 m and 0.15 m, respectively, which met the precision requirements for harvesting tasks. In the greenhouse peach orchard, FCR-HATS demonstrated notable robustness and the ability to operate continuously in real harvesting scenarios. Beyond agriculture, CRCM has the potential to be extended to collaborative tasks in construction and industrial logistics.
### CRCM Demo

Here is a demo of how the project works:

![Demo GIF](CRCM.gif)

This gif demonstrates the core functionality of CRCM：FTP generation module、Precise stopping module with collaborative region shrinkage、and Monitoring and re-following module.

## Installation

### Prerequisites

List any software or tools required to run the project.

- Python >= 3.9
- numpy >= 1.26.4
- pandas >= 2.2.3

### Installation Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/xiaowangzi6668/CRCM.git
