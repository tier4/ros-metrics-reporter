{{ $json := .Get "json" }}
{{ $height := .Get "height" | default "200px" }}
<div id="{{$json}}" class="plotly" style="height:{{$height}}"></div>
<script>
json_url = {{$.Site.BaseURL}} + {{$json}}
Plotly.d3.json(json_url, function(err, fig) {
    Plotly.plot('{{$json}}', fig.data, fig.layout, {responsive: true});
});
</script>
