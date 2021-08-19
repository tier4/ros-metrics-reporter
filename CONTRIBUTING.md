# How to contribute

Thanks for your interest in contributing to this project!

## Create an issue

* If you have a question, please ask it on issue page.
* If you want to report a bug, please create an issue.
* If you want to request a feature, please create an issue.

## Create a pull request

Pull requests are welcome :+1:

 1. Fork this repository.
 2. Create a new branch.
 3. Commit your changes.
 4. Push to your fork.
 5. Create a pull request.
 6. Check artifacts page to see your change.

## Run tests locally

Before running tests, you need to install dependencies. Check `Install dependencies` section at [action.yml](action.yml) and install them.
After that, you can run tests with the following command:

```sh
$ cd ros-metrics-reporter/scripts
$ python3 ../tests/run_local.py
$ cd ros-metrics-reporter/hugo-test
$ hugo server -D
$ firefox http://localhost:1313/
```
