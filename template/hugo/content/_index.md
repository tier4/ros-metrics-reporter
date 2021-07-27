---
title: Code quality dashboard
geekdocCollapseSection: true
geekdocNav: false
description: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
geekdocDescription: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
---

<details><summary>Legend</summary><div>

### Code coverage

<span class="LegendNA" title="Coverage rates are not available">N/A</span>
<span class="LegendLo" title="Coverage rates below [[ coverage_med ]] % are classified as low">low: &lt; [[ coverage_med ]] %</span>
<span class="LegendMed" title="Coverage rates between [[ coverage_med ]] % and [[ coverage_hi ]] % are classified as medium">medium: &gt;= [[ coverage_med ]] %</span>
<span class="LegendHi" title="Coverage rates of [[ coverage_hi ]] % and more are classified as high">high: &gt;= [[ coverage_hi ]] %</span>

### Code metrics

* CCN (Cyclomatic Complexity Number)  
<span class="LegendLo" title="Metrics rates below [[ ccn_recommendation ]] are classified as low">low: &lt; [[ ccn_recommendation ]]</span>
<span class="LegendMed" title="Metrics rates between [[ ccn_recommendation ]] and [[ ccn_threshold ]] are classified as medium">medium: &gt;= [[ ccn_recommendation ]]</span>
<span class="LegendHi" title="Metrics rates of [[ ccn_threshold ]] and more are classified as high">high: &gt;= [[ ccn_threshold ]]</span>

* LOC (Lines of Code)  
<span class="LegendLo" title="Metrics rates below [[ loc_recommendation ]] are classified as low">low: &lt; [[ loc_recommendation ]]</span>
<span class="LegendMed" title="Metrics rates between [[ loc_recommendation ]] and [[ loc_threshold ]] are classified as medium">medium: &gt;= [[ loc_recommendation ]]</span>
<span class="LegendHi" title="Metrics rates of [[ loc_threshold ]] and more are classified as high">high: &gt;= [[ loc_threshold ]]</span>

* Parameter count  
<span class="LegendLo" title="Metrics rates below [[ parameter_recommendation ]] are classified as low">low: &lt; [[ parameter_recommendation ]]</span>
<span class="LegendMed" title="Metrics rates between [[ parameter_recommendation ]] and [[ parameter_threshold ]] are classified as medium">medium: &gt;= [[ parameter_recommendation ]]</span>
<span class="LegendHi" title="Metrics rates of [[ parameter_threshold ]] and more are classified as high">high: &gt;= [[ parameter_threshold ]]</span>

</div></details>

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
