This is the design document for the [`ros-metrics-reporter`](../../ros-metrics-reporter) package.

# Purpose / Use cases

This package is intended to measure software quality and currently supports:

* LCOV coverage report
* Lizard metrics report
* Clang-Tidy report

# Design

This package is available on GitHub Actions. Even if the developer is not familiar with measuring software quality, you can use the right tools to measure software quality. It also supports dashboard functions and data logging that cannot be obtained when using each tool alone.

## Assumptions / Known limitations

If you have a lot of code in your repository, you may get stuck in GitHub Actions resource limits.
You can check the resource limit here:
<https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners#supported-runners-and-hardware-resources>

## Inputs / Outputs / API

### Input

You can get the input from the following sources:
<https://github.com/tier4/ros-metrics-reporter#available-options>

### Output

You will get an HTML page like this.
<https://tier4.github.io/ros-metrics-reporter/>

# References / External links

* LCOV: <http://ltp.sourceforge.net/coverage/lcov.php>
* Lizard: <https://github.com/terryyin/lizard>
* Clang-Tidy: <https://clang.llvm.org/extra/clang-tidy/>

# Future extensions / Unimplemented parts

* Addition of dynamic analysis tools

# Related issues
