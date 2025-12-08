# Feature Detection Method Selector - Implementation Summary

## Overview
Added a dropdown menu (QComboBox) to the RViz playback panel that allows users to dynamically change the feature detection method used in visual odometry calculations.

## Changes Made

### 1. Header File (`rosbag2_panel.hpp`)
- Added `#include <QComboBox>` for the dropdown widget
- Added new member variables:
  - `QComboBox* method_selector_` - The dropdown menu for method selection
  - `QLabel* method_label_` - Label for the dropdown
- Added new slot: `void onMethodChanged(int index)` - Handles method selection changes

### 2. Implementation File (`rosbag2_panel.cpp`)

#### UI Components Added
- **Method Selection Layout**: A new horizontal layout containing:
  - Label: "Feature Detection Method:"
  - Dropdown with options: SIFT, SURF, ORB, HOG
  - Default selection: SURF (index 1)
  - Tooltip: "Select the feature detection method for visual odometry"

#### Functionality Added
- **Shared VO Calculator Instance**: Created a file-scoped static function `get_vo_calculator()` that returns a reference to a single shared `VisualOdometry` instance. This ensures that:
  - The same instance is used across both `onChooseMomentClicked()` and `onMethodChanged()`
  - Method changes persist between calculations
  
- **Method Change Handler**: `onMethodChanged(int index)` function that:
  - Retrieves the selected method name from the dropdown
  - Calls `vo_calculator.change_method()` to switch the feature detection algorithm
  - Logs the change to the ROS logger

- **Configuration Persistence**: 
  - `load()` function now restores the previously selected method index
  - `save()` function now saves the current method index to the RViz configuration
  - This ensures the user's method preference persists across RViz sessions

## Available Methods

The dropdown provides four feature detection methods:

1. **SIFT** (Scale-Invariant Feature Transform)
   - Floating-point descriptors
   - Uses KD-Tree FLANN matcher
   - Good for general-purpose feature matching

2. **SURF** (Speeded-Up Robust Features) - Default
   - Floating-point descriptors
   - Uses KD-Tree FLANN matcher
   - Faster than SIFT with similar performance

3. **ORB** (Oriented FAST and Rotated BRIEF)
   - Binary descriptors
   - Uses LSH (Locality-Sensitive Hashing) FLANN matcher
   - Very fast, good for real-time applications

4. **HOG** (Histogram of Oriented Gradients)
   - Uses GFTT (Good Features To Track) for detection
   - Computes HOG descriptors at keypoint locations
   - Uses KD-Tree FLANN matcher

## Usage

1. Launch RViz with the playback panel
2. Load a bag file using the "Browse" button
3. Select your desired feature detection method from the dropdown
4. Click "Choose Moment" to perform visual odometry calculations
5. The selected method will be used for all subsequent calculations
6. Your method selection will be saved when you save the RViz configuration

## Technical Details

- The `VisualOdometry` class's `change_method()` function handles the switching between different feature detectors and their corresponding FLANN matchers
- The method can be changed at any time, even during playback
- Changes take effect immediately for the next calculation
- The implementation ensures thread-safety by using a single static instance accessed through a helper function
