<p><img width="800" alt="ROS Metrics Reporter" src="https://user-images.githubusercontent.com/19993104/129997676-4700c2eb-62af-4db1-9d22-d465d43b919d.png"></p>

[![Test github action](https://github.com/tier4/ros-metrics-reporter/actions/workflows/test-action.yml/badge.svg)](https://github.com/tier4/ros-metrics-reporter/actions/workflows/test-action.yml)
[![GitHub pages](https://img.shields.io/badge/-GitHub%20pages-orange)](https://tier4.github.io/ros-metrics-reporter/)

### [Demo](https://tier4.github.io/ros-metrics-reporter/)

## Overview

`ros-metrics-reporter` collects software metrics for the ROS packages of a project and generates HTML reports. The results can be saved as github-pages or artifacts.
This project is using [LCOV](https://github.com/linux-test-project/lcov), [Lizard](https://github.com/terryyin/lizard), [Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/), [CodeChecker](https://github.com/Ericsson/codechecker) and [Hugo](https://gohugo.io/) as backend. I would like to express my deepest gratitude for their contributions.

***Warning: the results will include your source code, so be careful about the scope of publication if you have a private repository. Even if your project is private, the scope of the GitHub Pages will be public. You can see more information [here](https://docs.github.com/en/pages/getting-started-with-github-pages/changing-the-visibility-of-your-github-pages-site).***

## Action setup

### Create orphan branch (First time only)

Before running this job, you need to create orphan branch.
NOTE: Replace `your-project` with your project name before running commands below.

```sh
git clone https://github/your-project.git
cd your-project
mkdir data
cd data
git init
git remote add origin https://github/your-project.git
touch .gitignore
git add .
git commit -m 'initial commit'
git push origin master:data
```

### Example workflow

Copy following yaml file to `.github/workflows/generate-metrics-report.yml` in your project.
Modify env vars to your needs.

```yml
name: Generate metrics report

on:
  workflow_dispatch:
  pull_request:
  push:
    branches: main

jobs:
  action-test:
    runs-on: ubuntu-latest
    env:
      ARTIFACTS_DIR: data
      BASE_URL: "https://tier4.github.io/ros-metrics-reporter/"
      TITLE: "ros2/demos"
      ROS_DISTRO: foxy
      VCS_FILE: ""  # Set vcs repo file if you need

    steps:
    - name: Clone default branch
      uses: actions/checkout@v2

    - name: Clone data branch in this repo
      uses: actions/checkout@v2
      with:
        ref: data
        path: ${{ env.ARTIFACTS_DIR }}

    - uses: ros-tooling/setup-ros@v0.2
      with:
        required-ros-distributions: ${{ matrix.ros_distribution }}

    - name: Build
      run: |
        . /opt/ros/${{ matrix.ros_distribution }}/setup.sh
        colcon build --event-handlers console_cohesion+ \
          --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" -DCMAKE_C_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

    - id: metrics-reporter
      uses: tier4/ros-metrics-reporter@v0.3
      with:
        artifacts-dir: ${{ env.ARTIFACTS_DIR }}
        base-url: ${{ env.BASE_URL }}
        title: ${{ env.TITLE }}
        ros-distro: ${{ env.ROS_DISTRO }}

    - name: Push artifacts
      if: github.ref == 'refs/heads/main'
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: ${{ env.ARTIFACTS_DIR }}
        publish_branch: data

    - name: Deploy public to gh-pages (main branch only)
      if: github.ref == 'refs/heads/main'
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: public
```

## Available options

| Option | Default Value | Description | Required | Example |
| :----- | :------------ | :---------- | :------- | :------ |
| `artifacts-dir` | N/A | Path to Artifacts generated using this Action (must include lcov-result/ and lizard-result/ directory). | `true` | `"${GITHUB_WORKSPACE}/doc"` |
| `target-dir` | `"${GITHUB_WORKSPACE}"` | Path to source directory. | `false` | `"${GITHUB_WORKSPACE}/src"` |
| `base-url` | N/A | If you use example/hugo-site in this repository, please specify the baseURL. | `true` | `"https://tier4.github.io/ros-metrics-reporter/"` |
| `title` | N/A | If you use example/hugo-site in this repository, please specify the title. | `true` | `"ros2/demos"` |
| `exclude` | N/A | Space separated list of exclude paths. | `false` | `"**/vendor/*"` |
| `ros-distro` | `"foxy"` | ROS distribution. | `false` | `"foxy"` |
| `extra-cmake-args` | `""` | Additional flags passed to CMake (using colcon build --cmake-args) | `false` | `"-DEXTRA_OPTION=ON"` |
| `hugo-dir` | `"${GITHUB_ACTION_PATH}/example/hugo-site"` | If you want to use your own hugo-site, specify the root directory. | `false` | `"${GITHUB_WORKSPACE}/hugo-site"` |
| `output-dir` | `"${GITHUB_WORKSPACE}/public"` | Hugo output directory. | `false` | `"${GITHUB_WORKSPACE}/output-dir"` |
| `lcovrc-path` | `"${GITHUB_ACTION_PATH}/.lcovrc"` | Path to .lcovrc file. | `false` | `"${GITHUB_WORKSPACE}/.lcovrc"` |
| `CCN` | `"15"` | Threshold for cyclomatic complexity number warning. | `false` | `"20"` |
| `CCN-recommendation` | `"5"` | Recommend value for cyclomatic complexity number. | `false` | `"10"` |
| `nloc` | `"1000000"` | Threshold for LOC (Lines Of Code). | `false` | `"200"` |
| `nloc-recommendation` | `"1000"` | Recommend value for LOC (Lines Of Code). | `false` | `"150"` |
| `arguments` | `"100"` | Limit for number of parameters. | `false` | `"100"` |
| `arguments-recommendation` | `"50"` | Recommend value for number of parameters. | `false` | `"50"` |
| `codechecker-config-path` | `"${GITHUB_ACTION_PATH}/codechecker-config.json"` | Path to codechecker-config.json file. | `false` | `"codechecker-config.json"` |
| `codechecker-skip-list` | `"${GITHUB_ACTION_PATH}/codechecker-skip-list.txt"` | Path to codechecker-skip-list. | `false` | `"codechecker-skip-list.txt"` |
| `target-repository` | `"${GITHUB_REPOSITORY}"` | If the repository you are measuring and the repository where you are running CI are different, specify the name of the repository you are measuring. | `false` | `"tier4/ros-metrics-reporter"` |
| `github-access-token` | N/A | If the repository to be measured is private, specify a token. | `false` | `"ghp_DYwodLrS87q3aaagwerjgowaeiogjawrega"` |
