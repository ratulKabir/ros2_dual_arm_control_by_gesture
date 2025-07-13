# IMPROVEMENTS

## Research & Resources
- **Issue:** Research work was limited due to time constraints. Resources were chosen mainly for code availability, results, and recency.
- **Improvement:** Select resources based on multiple factors:
    - Project requirements
    - Author group credibility
    - Documentation quality
    - Licensing compatibility
    - Technology fit etc.

## Docker Image
- **Improvement:** Update the Docker image to include all required libraries. Push the updated image for reusability.

## Hand Tracking
- **Issue:** Tracking is jittery and hands are often swapped.
- **Improvements:**
    - Use more robust hand tracking methods tailored for this use case.
    - Fine-tune pretrained models on in-house data.
    - Improve pixel-to-robot 2D space conversion with advanced approaches.

## ROS2 Node Architecture
- **Improvement:** Research better ways to publish hand poses. Consider using separate publishers for each hand to control two robots.

## Xacro Usage
- **Issue:** Xacro caused unexpected robot movement in MoveIt2 config; root cause not found.
- **Improvement:** Investigate and resolve Xacro issues. Use Xacro instead of URDF for better reusability and maintainability.

## ROS2 Launch & Configuration
- **Improvement:** Add launch files for all ROS2 nodes. Use YAML configuration for parameters to improve efficiency.

## Robot Movement
- **Improvements:**
    - Apply trajectory smoothing or denoising methods.
    - Classify tasks based on hand gesture intention (e.g., "Reach pen", "Drop banana in bin") and generate target points accordingly.
    - Integrate prior intention knowledge for smoother robot movement.
    - Implement MPC or Kalman Filter-based planners for smoother motion.
    - Use uncertainty estimation (e.g., MC dropout) to adjust robot reaction speed to new targets.

## Error Investigation
- **Issues:**
    - "Target point invalid" errors occur despite valid points.
    - Two nodes cannot always operate simultaneously.
- **Improvement:** Investigate and resolve these issues.

## Metrics & Visualization
- **Improvements:**
    - Develop metrics for speed and reliability.
    - Implement trajectory visualization.
    - Use displacement error metrics if ground truth is available.

## Gripper Action
- **Issue:** Gripper status detected but not used.
- **Improvement:** Integrate gripper actions into robot control.

## General Improvements
- Adopt better naming conventions.
- Improve ROS2 package maintenance.
- Follow best git practices.
- Enhance Python project structure and handling.
- For deep learning data collection:
    - Ensure system robustness.
    - Remove hard-coded normalization.
    - Enable automatic adaptation to different settings (e.g., image size).