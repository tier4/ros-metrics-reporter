This is the design document for the [`ros-metrics-reporter`](../../ros-metrics-reporter) package.

# Purpose / Use cases

This package is intended to measure software quality for any ROS2 package. The metrics are collected by [`ros-metrics-reporter`](../../ros-metrics-reporter) and the results are generated as HTML reports.
Currently, the following metrics are collected:

* LCOV coverage report
  * Includes Line, Function and Branch coverage.
  * You can specify test labels to filter the coverage report.
* Lizard metrics report
* Clang-Tidy report

Since these tools are not dedicated to ROS2, if you want results for each package, you need to process the results.
The `ros-metrics-reporter` automates this tedious procedure and outputs the information that the user really wants to know.

## Description of metrics

This section describes the various metrics and coverage results collected by `ros-metrics-reporter`.

### Code coverage

Code coverage calculates the percentage of source code that is covered by unit tests and integration tests.
Line coverage calculates the coverage of the code in lines, Function coverage calculates the coverage of the code in functions, Branch coverage calculates the coverage of branches such as if statements and for loops.
The results are output in a tabular format for each package.

![coverage table](img/coverage_table.png)

The background color will be colored according to the coverage rate of the code. You can set a recommended value and a threshold value for the coverage. By default, the recommended value is 90% and the threshold is 75%.
If it exceeds the recommended value, it will be displayed in green. If the value is between the recommended value and the threshold, it will be displayed in yellow. If it is below the threshold, it is displayed in red.
The legend is displayed at the top.

![coverage threshold](img/coverage_threshold.png)

`ros-metrics-reporter` also generates a time series graph by referring to the code coverage values measured in the past.

![coverage graph](img/coverage_graph.png)

Comparison with past code coverage will help you to see if the code quality of your project is improving.
The graph is also generated for each test label, which will be explained later. You can also compare the detailed values for each test label.
From the link below the graph, you can access the HTML report by `LCOV`. This report contains detailed coverage information on a per-file basis. Therefore, you can see which lines are not covered by the current test suite.

#### About test labels

The `colcon test` command runs the entire test suite. This includes unit tests with `gtest` and integration tests with `launch_testing`. Normally, if you want to measure code coverage, you run `colcon test`, so the code coverage you get is the result of running all tests, including unit tests and integration tests. However, in order to improve the quality of your tests, you need to make sure that your code is covered by meaningful tests, both unit and integration tests.
For this reason, `ros_metrics_reporter` provides the ability to measure code coverage in unit tests alone and code coverage in integration tests alone.

#### How to add test labels to your code

In the case of `ament_add_gtest()`, the test label is set to `gtest`.
In the case of `add_launch_test()` or other testing tools, you can use `set_tests_properties()` to set the test label as follows.

```cmake
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/my_launch_test.test.py
    TARGET my_launch_test)
  set_tests_properties(my_launch_test PROPERTIES LABELS "launch_test")
```

### Code metrics

There are three metrics that can be measured with `ros_metrics_reporter`: CCN (Cyclomatic Complexity Number), LOC (Lines of Code), and Parameter count.
CCN represents the complexity of the code. It measures the number of linearly independent paths in the code. Therefore, if there are more branches such as if and for statements, the CCN increases; an increase in CCN means an increase in the complexity of the code, which leads to an increase in the probability of bugs and a decrease in maintainability.
LOC is the number of lines of code contained in a single function. An increase in the number of lines of code is considered to be a bad thing because it impairs readability.
Parameter count represents the number of arguments to a function. More arguments in a function indicates a larger scope of the function. To improve maintainability, it is important to make the scope of one function smaller and to reduce the number of arguments.

The results are output in a tabular format for each package.

![metrics table](img/metrics_table.png)

The background color will be colored according to the coverage rate of the code. You can set a recommended value and a threshold value for the coverage. By default, the recommended value is 90% and the threshold is 75%.
If it exceeds the recommended value, it will be displayed in green. If the value is between the recommended value and the threshold, it will be displayed in yellow. If it is below the threshold, it is displayed in red.
The legend is displayed at the top.

![metrics threshold](img/metrics_threshold.png)

`ros-metrics-reporter` also generates time series graphs by referring to the values of metrics measured in the past. Comparison with past metrics will help you to see if the code quality of your project is improving or not.

![metrics graph](img/metrics_graph.png)

From the link below the graph, you can access the HTML report by `Lizard`. This report contains detailed metrics information on a per-function basis.

### Static analysis

From the link in the Clang-Tidy section, you can access the HTML report by `Clang-Tidy`. This report contains tips on how to improve your code.

![static analysis](img/static_analysis.png)

### Top contributors

When creating a pull request, you may want to ask the developer of the package a question or request a review. If the package metadata contains the correct information about the developers, that's fine, but if not, you will need to find the contributors in the commit history and contact them. In this situation, the Top 3 contributors feature is useful to know who has committed a lot. You can look for contributors and contact them without referring to the commit history.

### Code frequency graph

Code change history is an indicator of how active the development of a package is. If the metrics and code coverage are not degraded despite frequent changes to the package, the development is in an ideal state.

# Design

This package is available on GitHub Actions. Even if the developer is not familiar with measuring software quality, you can use the right tools to measure software quality. It also supports dashboard functions and data logging that cannot be obtained when using each tool alone.

## File structure

### action.yml

#### inputs section

The inputs section is used to specify the input files and variables.

#### outputs section

The outputs section is used to specify the output directory. It is useful when you want to deploy the results to a remote server on GitHub Actions.

#### runs section

The runs section describes the steps to generate metrics reports.

### Python scripts

* The entrypoint file is `ros_metrics_reporter/ros_metrics_reporter.py`.
* Tests are in `tests/`.

### Flowchart

![Flowchart](img/ros_metrics_reporter_flowchart.drawio.svg)

## Assumptions / Known limitations

If you have a lot of code in your repository, you may get stuck in GitHub Actions resource limits.
You can check the resource limit here:
<https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources>

## Inputs / Outputs / API

### Input

#### Arguments

You can get the input from the following sources:
<https://github.com/tier4/ros-metrics-reporter#available-options>

#### Configuration files

##### LCOV

**.lcovrc**
You can copy this file to your repository and edit the file to change the default options. Available options are: <https://linux.die.net/man/5/lcovrc>
You need to locate the file in the root directory of your repository if you edit the file.

##### CodeChecker

**codechecker-config.json**
You can copy this file to your repository and edit the file to change the default options. Available options are: <https://codechecker.readthedocs.io/en/latest/analyzer/user_guide/#analyzer-configuration-file>
You need to specify the file with the `codechecker-config-path` argument.

**codechecker-skip-list.txt**
You can copy this file to your repository and edit the file to change the exclude settings. File format is described here: <https://codechecker.readthedocs.io/en/latest/analyzer/user_guide/#skip>

##### Hugo

By overriding the Hugo configuration files and themes in this repository, you can use the design and settings of your choice.
The configuration files and references are as follows:

**example/hugo-site/config.toml**
 <https://gohugo.io/getting-started/configuration/#configuration-file>

**example/hugo-site/themes/**
The various themes can be downloaded from the following link: <https://themes.gohugo.io/>

### Output

You will get an HTML page like this.
<https://tier4.github.io/ros-metrics-reporter/>

# Usage

## Basic usage of `ros-metrics-reporter`

### Create new branch (First time only)

Before running this job, you need to create orphan branch.
This tool automatically pushes artifacts to the data branch of your repository every time. The old data will be used to generate a time series graph on the HTML artifact.
To generate new branch, you need to run the following command.
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

### Write your GitHub Actions

In order to create a correct HTML report, please follow the steps below.

### Step1: Clone target repository

Clones your repository, using the `ref` option to specify the branch to be cloned. The default branch is the one configured in your repository settings.

```yaml
    - name: Clone default branch
      uses: actions/checkout@v2
```

Also, clone the data branch created in the above procedure.

```yaml
    - name: Clone data branch in this repo
      uses: actions/checkout@v2
      with:
        ref: data
        path: ${{ env.ARTIFACTS_DIR }}
```

### Step2: Setup ROS environment

Setup ROS environment, using the `required-ros-distributions` option to specify the ROS distribution to use.

```yaml
    - uses: ros-tooling/setup-ros@v0.2
      with:
        required-ros-distributions: humble
```

### Step3: Build

Copy the following code to the `runs` section of your workflow. The build option is important for generating the test coverage report.

```yaml
    - name: Build
      run: |
        . /opt/ros/humble/setup.sh
        colcon build --event-handlers console_cohesion+ \
          --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" -DCMAKE_C_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Step4: Generate coverage report

Copy the following code to the `runs` section of your workflow. Add or modify input arguments as needed.

```yaml
    - id: metrics-reporter
      uses: tier4/ros-metrics-reporter@v0.3
      with:
        artifacts-dir: ${{ env.ARTIFACTS_DIR }}
        base-url: ${{ env.BASE_URL }}
        title: ${{ env.TITLE }}
        ros-distro: ${{ env.ROS_DISTRO }}
```

### Step5: Push artifacts

Push the metrics data to the data branch to be used for the next time series graph creation.

```yaml
    - name: Push artifacts
      if: github.ref == 'refs/heads/main'
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: ${{ env.ARTIFACTS_DIR }}
        publish_branch: data
```

If you want to deploy the HTML artifacts to a GitHub Pages, you need to add the following code to the `runs` section of your workflow.

```yaml
    - name: Deploy public to gh-pages (main branch only)
      if: github.ref == 'refs/heads/main'
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: public
```

### Generated branches

**data**: The data branch is used to generate the time series graph.

**gh-pages**: The gh-pages branch is used to deploy the HTML artifacts to a GitHub pages.

# Development

## Setup development environment

You can use venv to setup development environment.

```sh
cd ros-metrics-reporter
python3 -m venv venv
source venv/bin/activate
```

After the environment is set up, you can install the dependencies with Poetry.

```sh
pip3 install poetry
poetry install
```

If you want to run the tests, you can use the following command.

```sh
poetry run tox
```

# References / External links

* LCOV: <http://ltp.sourceforge.net/coverage/lcov.php>
* Lizard: <https://github.com/terryyin/lizard>
* Clang-Tidy: <https://clang.llvm.org/extra/clang-tidy/>

# Future extensions / Unimplemented parts

* Addition of dynamic analysis tools
