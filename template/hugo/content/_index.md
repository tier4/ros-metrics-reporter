---
title: Code quarity dashboard
geekdocCollapseSection: true
---

| Package name | Coverage rate | Metrics |
| :---: | :---: | :---: |
{%- for package in packages %}
| {{ package }} | __COVERAGE_BADGE__ | __METRICS_BADGE__ |
{%- endfor %}
