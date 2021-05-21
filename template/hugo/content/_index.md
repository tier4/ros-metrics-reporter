---
title: Code quarity dashboard
geekdocCollapseSection: true
---

<table>
<tr><td> Package name <td> Line coverage [%] <td> Functions [%] <td> Branches [%] <td> CCN violation count <td> LOC violation count <td> Parameter violation count
{%- for param in param_list %}
<tr><td> {{ param.package }} {{ param.line_badge }} {{ param.functions_badge }} {{ param.branches_badge }} {{ param.ccn_badge }} {{ param.loc_badge }} {{ param.parameter_badge }}
{%- endfor %}
</table>
