---
title: [[ package_name ]]
geekdocNav: false
---

[[ package_name ]]

---

# Test result

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
