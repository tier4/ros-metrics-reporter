---
title: Code quarity dashboard
geekdocCollapseSection: true
---

| Package name | Line coverage [%] | CCN violation |
| :---: | :---: | :---: |
{%- for param in param_list %}
| {{ param.package }} | {{ param.coverage_badge }} | {{ param.metrics_badge }} |
{%- endfor %}
