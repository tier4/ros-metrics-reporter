---
title: Code quarity dashboard
geekdocCollapseSection: true
geekdocNav: false
geekdocDescription: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
---

<div class="wrapped">
<table>
<tr><td> Package name <td> Line coverage [%] <td> Functions [%] <td> Branches [%] <td> CCN (over recommended value) <td> CCN (over required value) <td> CCN (worst value) <td> LOC (over recommended value) <td> LOC (over required value) <td> LOC (worst value) <td> Parameter (over recommended value) <td> Parameter (over required value) <td> Parameter (worst value)
{%- for param in param_list %}
<tr><td> [[ param.package ]] [[ param.line_badge ]] [[ param.functions_badge ]] [[ param.branches_badge ]] [[ param.ccn_warning_badge ]] [[ param.ccn_violation_badge ]] [[ param.ccn_worst_badge ]] [[ param.loc_warning_badge ]] [[ param.loc_violation_badge ]] [[ param.loc_worst_badge ]] [[ param.parameter_warning_badge ]] [[ param.parameter_violation_badge ]] [[ param.parameter_worst_badge ]]
{%- endfor %}
</table>
</div>

---

# Test result for all packages

## Code coverage

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_lcov_figure_name ]]" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code coverage here.]([[ lcov_result_html_link ]])
{{< /hint >}}

## Code metrics

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_ccn_figure_name ]]" height="400px" >}}

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_loc_figure_name ]]" height="400px" >}}

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_parameter_figure_name ]]" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code metrics here.]([[ lizard_result_html_link ]])
{{< /hint >}}

## Clang-Tidy

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access the Clang-Tidy results here.]([[ tidy_result_html_link ]])
{{< /hint >}}
