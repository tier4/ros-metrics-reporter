---
title: [[ package_name ]]
geekdocNav: false
description: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
geekdocDescription: Metrics reporter is a tool for continuously monitoring various software metrics. For more information, please visit https://github.com/tier4/ros-metrics-reporter .
---

[[ package_name ]]

---

# Test result

## Code coverage

{%- for test_label in test_label_list %}

### Test label: [[ test_label.name ]]

{{< load-plotly >}}
{{< plotly json="plotly/[[ test_label.plotly_lcov_figure_name ]]" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code coverage here.]([[ test_label.lcov_result_html_link ]])
{{< /hint >}}

{%- endfor %}

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

## Code frequency

<figure class="activity">

<div class="activity-graph">

{{< load-plotly >}}
{{< plotly json="plotly/[[ package_name ]]/code_frequency_graph.json" height="400px" >}}

</div>
<div class="activity-user">
<table>
<tr><th colspan="3"></th></tr>
<tr><th colspan="3"><h2>Top3 contributors</h2></th></tr>
<tr>
<td><h3>1.</h3></td>
<td><h3>[[ contributor_name_1 ]]</h3></td>
<td><h3>[[ contribute_count_1 ]]</h3></td>
</tr>
<tr>
<td><h3>2.</h3></td>
<td><h3>[[ contributor_name_2 ]]</h3></td>
<td><h3>[[ contribute_count_2 ]]</h3></td>
</tr>
<tr>
<td><h3>3.</h3></td>
<td><h3>[[ contributor_name_3 ]]</h3></td>
<td><h3>[[ contribute_count_3 ]]</h3></td>
</tr>
</table>
</div>
</div>
