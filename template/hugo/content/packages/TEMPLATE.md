---
title: "__TEMPLATE__"
---

__TEMPLATE__

---

# Test result

## Code coverage

{{< load-plotly >}}
{{< plotly json="plotly/__PLOTLY_LCOV_FIGURE_NAME__" height="400px" >}}

[You can access more detailed data on code coverage here.](__LCOV_RESULT_HTML_LINK__)

## Code metrics

{{< load-plotly >}}
{{< plotly json="plotly/__PLOTLY_METRICS_FIGURE_NAME__" height="400px" >}}

[You can access more detailed data on code metrics here.](__LIZARD_RESULT_HTML_LINK__)
