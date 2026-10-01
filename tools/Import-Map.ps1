param([string]$Endpoint = 'https://api.openstreetmap.org/api/0.6/map')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$route = Get-Content "$root/data/route_108.json" -Raw | ConvertFrom-Json
$points = @($route.directions | ForEach-Object { $_.points | ForEach-Object { ,$_ } })
$longitudes = @($points | ForEach-Object { $_[0] })
$latitudes = @($points | ForEach-Object { $_[1] })
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$south = (($latitudes | Measure-Object -Minimum).Minimum - 0.002).ToString($culture)
$north = (($latitudes | Measure-Object -Maximum).Maximum + 0.002).ToString($culture)
$west = (($longitudes | Measure-Object -Minimum).Minimum - 0.003).ToString($culture)
$east = (($longitudes | Measure-Object -Maximum).Maximum + 0.003).ToString($culture)
$bbox = "$south,$west,$north,$east"
$nodes = @{}
$ways = @{}
for ($column = 0; $column -lt 3; $column++) {
	for ($row = 0; $row -lt 3; $row++) {
		$left = [double]::Parse($west, $culture) + $column * ([double]::Parse($east, $culture) - [double]::Parse($west, $culture)) / 3
		$bottom = [double]::Parse($south, $culture) + $row * ([double]::Parse($north, $culture) - [double]::Parse($south, $culture)) / 3
		$right = $left + ([double]::Parse($east, $culture) - [double]::Parse($west, $culture)) / 3
		$top = $bottom + ([double]::Parse($north, $culture) - [double]::Parse($south, $culture)) / 3
		$tile = (@($left, $bottom, $right, $top) | ForEach-Object { $_.ToString($culture) }) -join ','
		$xml = Invoke-RestMethod ($Endpoint + "?bbox=$tile") -TimeoutSec 120
		foreach ($node in $xml.osm.node) {
			$nodes[[string]$node.id] = @{ lon = [double]::Parse($node.lon, $culture); lat = [double]::Parse($node.lat, $culture) }
		}
		foreach ($way in $xml.osm.way) { $ways[[string]$way.id] = $way }
		Write-Host "Map tile $($column * 3 + $row + 1)/9"
	}
}
$elements = @(
	foreach ($way in ($ways.Values | Sort-Object { [long]$_.id })) {
		$tags = @{}
		foreach ($tag in $way.tag) { $tags[[string]$tag.k] = [string]$tag.v }
		if (!$tags.ContainsKey('building') -and !$tags.ContainsKey('highway') -and $tags.natural -ne 'water') { continue }
		if ($tags.highway -in @('footway', 'path', 'steps', 'cycleway', 'pedestrian')) { continue }
		$geometry = @($way.nd | ForEach-Object { $nodes[[string]$_.ref] })
		if ($geometry.Count -lt 2 -or $geometry -contains $null) { continue }
		@{ id = [long]$way.id; tags = $tags; geometry = $geometry }
	}
)
if ($elements.Count -lt 10) { throw 'Incomplete OSM map response.' }
$data = [ordered]@{ fetched_at = (Get-Date).ToUniversalTime().ToString('o'); source = $Endpoint; attribution = '(c) OpenStreetMap contributors, ODbL 1.0'; bounds = $bbox; elements = $elements }
$data | ConvertTo-Json -Depth 20 -Compress | Set-Content "$root/data/map_108.json" -Encoding utf8
Write-Host "Imported $($elements.Count) OSM ways."