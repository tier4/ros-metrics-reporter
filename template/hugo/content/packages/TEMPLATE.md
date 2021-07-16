---
title: {{ package_name }}
geekdocNav: false
---

{{ package_name }}

---

# Test result

## Code coverage

{% raw %}
{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_lcov_figure_name }}{% raw %}" height="400px" >}}

{{< hint info >}}
{{< icon "gdoc_link" >}} [You can access more detailed data on code coverage here.]({% endraw %}{{ lcov_result_html_link }){% raw %})
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
