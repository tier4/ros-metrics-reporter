---
title: {{ package_name }}
---

{{ package_name }}

---

# Test result

## Code coverage

{% raw %}
{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_lcov_figure_name }}{% raw %}" height="400px" >}}
{% endraw %}

[You can access more detailed data on code coverage here.]({{ lcov_result_html_link }})

## Code metrics

{% raw %}
{{< load-plotly >}}
{{< plotly json="plotly/{% endraw %}{{ plotly_metrics_figure_name }}{% raw %}" height="400px" >}}
{% endraw %}

[You can access more detailed data on code metrics here.]({{ lizard_result_html_link }})
