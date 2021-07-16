---
title: Code quarity dashboard
geekdocCollapseSection: true
geekdocNav: false
---

<div class="wrapped">
<table>
<tr><td> Package name <td> Line coverage [%] <td> Functions [%] <td> Branches [%] <td> CCN (worst) <td> CCN (average) <td> CCN violation count <td> LOC violation count <td> Parameter violation count
{%- for param in param_list %}
<tr><td> {{ param.package }} {{ param.line_badge }} {{ param.functions_badge }} {{ param.branches_badge }} {{ param.ccn_worst_badge }} {{ param.ccn_average_badge }} {{ param.ccn_violation_badge }} {{ param.loc_badge }} {{ param.parameter_badge }}
{%- endfor %}
</table>
</div>

---

# Test result for all packages

## Code coverage

{% raw %}
{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_lcov_figure_name }}{% raw %}" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code coverage here.]({% endraw %}{{ lcov_result_html_link }}{% raw %})
{{< /hint >}}
{% endraw %}

## Code metrics

{% raw %}
{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_metrics_ccn_figure_name }}{% raw %}" height="400px" >}}

{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_metrics_loc_figure_name }}{% raw %}" height="400px" >}}

{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_metrics_parameter_figure_name }}{% raw %}" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code metrics here.]({% endraw %}{{ lizard_result_html_link }}{% raw %})
{{< /hint >}}
{% endraw %}

## Clang-Tidy

{% raw %}
{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access the Clang-Tidy results here.]({% endraw %}{{ tidy_result_html_link }}{% raw %})
{{< /hint >}}
{% endraw %}
