# Task 1

## Setup

You can create a Python virtual environment by installing the required packages listed in `requirements.txt`. Once installed, the code will run on any PC.

## Template Matching

* To run tracking on different sequences, update the path at the top of the script by using "Change All Occurrences".

## Object Detection & Association

* Similarly, to run tracking on different sequences, change the path at the beginning of the code using "Change All Occurrences".

## Evaluation (Template Matching & Object Detection)

* To evaluate different sequences, update the path at the start of the evaluation script (using "Change All Occurrences").
* The evaluation outputs plots of Success and Precision rates against IoU and center-error thresholds, averaged over 150 frames.
* You can modify `given_iou_threshold` and `given_error_threshold` at the beginning of the code to see Success and Precision rates for your chosen thresholds in the terminal.

## Visualization (Template Matching & Object Detection)

* To visualize a specific frame, change the path at the top of the visualization script.
* The output images show predicted and ground-truth bounding boxes for both TM and ODA methods.

## Improved Method

* Pretrained network files are provided in the `mobilenet_ssd/` folder.
* To run tracking on different sequences, change the path at the beginning of the code (use "Change All Occurrences").
* To evaluate different sequences, update the path in the evaluation script.
* Evaluation produces plots of Success and Precision rates against IoU and center-error thresholds, averaged over 150 frames for all three methods.
* Modify `given_iou_threshold` and `given_error_threshold` to see respective Success and Precision rates for all methods in the terminal.
* To visualize a specific frame, change the path at the top of the visualization script. The images show predicted vs. ground-truth bounding boxes for the improved method.

# Task 2

## Example Code (Homework Publisher)

These are the original code snippets provided; they have not been modified.

### Q1: Constant Velocity Model (CVM)

* Running the code automatically computes the mean ADE/FDE for future horizons of 3s, 5s, and 8s across all agents and scenarios.
* It also plots each scenario map with agents’ predicted and ground-truth trajectories, colored by time horizon.
* Plots are saved to `visualization_CV/`.

### Q2: Constant Acceleration Model (CAM)

* Running the code automatically computes the mean ADE/FDE for future horizons of 3s, 5s, and 8s across all agents and scenarios.
* It plots scenario maps with predicted vs. ground-truth trajectories, labeled by time horizon.
* Plots are saved to `visualization_CA/`.

### Q3: Discussion Visualizations

* **Map Visualization**: A script selects 10 random scenarios, plotting both CVM and CAM trajectories with ground truth on maps. Outputs are in `visualization_discussions/`.
* **Graph Plots**: A separate script selects 10 random scenarios, plotting CVM and CAM predicted vs. ground-truth trajectories on graphs for comparison.
* Both scripts include Jupyter Notebook discussions of the results.
