---
title: Code quarity dashboard
geekdocCollapseSection: true
---

| Package name | Line coverage [%] | Functions [%] | Branches [%] | CCN violation | LOC violation | Parameter violation |
| :----------: | :---------------: | :-----------: | :----------: | :-----------: | :-----------: | :-----------------: |
{%- for param in param_list %}
| {{ param.package }} | {{ param.line_badge }} | {{ param.functions_badge }} | {{ param.branches_badge }} | {{ param.ccn_badge }} | {{ param.loc_badge }} | {{ param.parameter_badge }} |
{%- endfor %}
