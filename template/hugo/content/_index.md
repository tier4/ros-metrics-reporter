---
title: Code quality dashboard
geekdocCollapseSection: true
geekdocNav: false
description: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
geekdocDescription: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
---

Last updated: [[ last_updated ]]

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
<thead>
<tr><th></th><th colspan="3" title="Test coverage is a measure used to describe the degree to which the source code of a program is executed when a particular test suite runs."><a href="https://www.bullseye.com/coverage.html" target="_blank" rel="noopener noreferrer"> Coverage </a></th><th colspan="9" title="A software metric is a standard of measure of a degree to which a software system or process possesses some property."> Metrics </th>
<tr><th></th><th colspan="3"></th><th colspan="3" title="Cyclomatic Complexity Number"><a href="https://en.wikipedia.org/wiki/Cyclomatic_complexity" target="_blank" rel="noopener noreferrer"> CCN </a></th><th colspan="3" title="Lines of Code"> LOC </th><th colspan="3" title="Parameter count"> Parameter </th>
<tr><th> Package name <th> Line [%] <th> Functions [%] <th> Branches [%] <th title="The number of functions that exceed the recommended value of the metrics."> Over recommended value <th title="The number of functions that exceed the threshold value of the metrics."> Over required value <th title="This is the worst value of the metrics measurements."> Worst value <th title="The number of functions that exceed the recommended value of the metrics."> Over recommended value <th title="The number of functions that exceed the threshold value of the metrics."> Over required value <th title="This is the worst value of the metrics measurements."> Worst value <th title="The number of functions that exceed the recommended value of the metrics."> Over recommended value <th title="The number of functions that exceed the threshold value of the metrics."> Over required value <th title="This is the worst value of the metrics measurements."> Worst value
</thead>
<tbody>
{%- for param in param_list %}
<tr><td> [[ param.package ]] [[ param.line_badge ]] [[ param.functions_badge ]] [[ param.branches_badge ]] [[ param.ccn_warning_badge ]] [[ param.ccn_violation_badge ]] [[ param.ccn_worst_badge ]] [[ param.loc_warning_badge ]] [[ param.loc_violation_badge ]] [[ param.loc_worst_badge ]] [[ param.parameter_warning_badge ]] [[ param.parameter_violation_badge ]] [[ param.parameter_worst_badge ]]
{%- endfor %}
</tbody>
</table>
</div>

---

# Test result for all packages

## Code coverage

{%- for test_label in test_label_list %}

### Test label: [[ test_label.name ]]

{{< load-plotly >}}
{{< plotly json="plotly/[[ test_label.plotly_lcov_figure_name ]]" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code coverage here.]([[ test_label.lcov_result_html_link ]])
{{< /hint >}}

{%- endfor %}

---

## Code metrics

### CCN

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_ccn_figure_name ]]" height="400px" >}}

### LOC

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_loc_figure_name ]]" height="400px" >}}

### Parameter

{{< load-plotly >}}
{{< plotly json="plotly/[[ plotly_metrics_parameter_figure_name ]]" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code metrics here.]([[ lizard_result_html_link ]])
{{< /hint >}}

---

## Clang-Tidy

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access the Clang-Tidy results here.]([[ tidy_result_html_link ]])
{{< /hint >}}

---

## Code frequency

<figure class="activity">

<div class="activity-graph">

{{< load-plotly >}}
{{< plotly json="plotly/all/code_frequency_graph.json" height="400px" >}}

</div>
<div class="activity-user">
<table>
<tr><th colspan="4"></th></tr>
<tr><th colspan="4"><h2>Top3 contributors</h2></th></tr>
<tr>
<td><h3>1.</h3></td>
<td class="avatar"><img class="avatar-image" src="[[ contributor_avatar_1 ]]"/></td>
<td><h3>[[ contributor_name_1 ]]</h3></td>
<td><h3>[[ contribute_count_1 ]]</h3></td>
</tr>
<tr>
<td><h3>2.</h3></td>
<td class="avatar"><img class="avatar-image" src="[[ contributor_avatar_2 ]]"/></td>
<td><h3>[[ contributor_name_2 ]]</h3></td>
<td><h3>[[ contribute_count_2 ]]</h3></td>
</tr>
<tr>
<td><h3>3.</h3></td>
<td class="avatar"><img class="avatar-image" src="[[ contributor_avatar_3 ]]"/></td>
<td><h3>[[ contributor_name_3 ]]</h3></td>
<td><h3>[[ contribute_count_3 ]]</h3></td>
</tr>
</table>
</div>
</div>
