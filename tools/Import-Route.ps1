param([string]$GtfsDirectory = '', [datetime]$ServiceDate = (Get-Date).Date)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$cache = Join-Path $root '.cache'
New-Item $cache -ItemType Directory -Force | Out-Null
$catalog = 'https://ckan.multimediagdansk.pl/api/3/action/package_show?id=tristar'
$source = $catalog
if (!$GtfsDirectory) {
    $resources = (Invoke-RestMethod $catalog).result.resources
    $source = ($resources | Where-Object id -eq '30e783e4-2bec-4a7d-bb22-ee3e3b26ca96').url
    Invoke-WebRequest $source -OutFile "$cache/gtfs.zip" -TimeoutSec 120
    Expand-Archive "$cache/gtfs.zip" "$cache/gtfs" -Force
    $GtfsDirectory = "$cache/gtfs"
}
$routes = Import-Csv "$GtfsDirectory/routes.txt"
$route = $routes | Where-Object route_short_name -eq '108' | Select-Object -First 1
if (!$route) { throw 'Route 108 is absent from this feed.' }
$dateKey = $ServiceDate.ToString('yyyyMMdd')
$weekday = $ServiceDate.DayOfWeek.ToString().ToLowerInvariant()
$activeServices = @{}
if (Test-Path "$GtfsDirectory/calendar.txt") {
    foreach ($calendar in (Import-Csv "$GtfsDirectory/calendar.txt")) {
        if ($calendar.start_date -le $dateKey -and $calendar.end_date -ge $dateKey -and $calendar.$weekday -eq '1') {
            $activeServices[$calendar.service_id] = $true
        }
    }
}
if (Test-Path "$GtfsDirectory/calendar_dates.txt") {
    foreach ($exception in (Import-Csv "$GtfsDirectory/calendar_dates.txt" | Where-Object date -eq $dateKey)) {
        if ($exception.exception_type -eq '1') { $activeServices[$exception.service_id] = $true }
        if ($exception.exception_type -eq '2') { $activeServices.Remove($exception.service_id) }
    }
}
if (!$activeServices.Count) { throw "No service in feed for $dateKey. Download a current GTFS archive." }
$trips = @(Import-Csv "$GtfsDirectory/trips.txt" | Where-Object { $_.route_id -eq $route.route_id -and $activeServices.ContainsKey($_.service_id) })
$tripIds = @{}
foreach ($trip in $trips) { $tripIds[$trip.trip_id] = $true }
$times = @(Import-Csv "$GtfsDirectory/stop_times.txt" | Where-Object { $tripIds.ContainsKey($_.trip_id) })
$stops = @{}
Import-Csv "$GtfsDirectory/stops.txt" | ForEach-Object { $stops[$_.stop_id] = $_ }
$shapes = Import-Csv "$GtfsDirectory/shapes.txt"
$directions = @()
foreach ($group in ($trips | Group-Object direction_id)) {
    $candidates = @{}
    foreach ($trip in $group.Group) { $candidates[$trip.trip_id] = $trip }
    $selected = $times | Where-Object { $candidates.ContainsKey($_.trip_id) } | Group-Object trip_id | Sort-Object Count -Descending | Select-Object -First 1
    $trip = $candidates[$selected.Name]
    $points = @($shapes | Where-Object shape_id -eq $trip.shape_id | Sort-Object { [int]$_.shape_pt_sequence } | ForEach-Object { ,@([double]$_.shape_pt_lon, [double]$_.shape_pt_lat) })
    $sequence = @($selected.Group | Sort-Object { [int]$_.stop_sequence } | ForEach-Object {
        $stop = $stops[$_.stop_id]
        [ordered]@{ id = $stop.stop_id; name = $stop.stop_name; lon = [double]$stop.stop_lon; lat = [double]$stop.stop_lat; arrival = $_.arrival_time; departure = $_.departure_time }
    })
    $sequence = @($sequence | Where-Object { $_.name -notlike 'Jana z Kolna*' })
    if ($points.Count -lt 20 -or $sequence.Count -lt 5) { throw 'Incomplete route geometry or stop sequence.' }
    $directions += [ordered]@{ id = $group.Name; headsign = $trip.trip_headsign; trip_id = $trip.trip_id; service_id = $trip.service_id; shape_id = $trip.shape_id; points = $points; stops = $sequence }
    Write-Host "$($trip.trip_headsign): $($points.Count) shape points, $($sequence.Count) stops"
}
if ($directions.Count -ne 2) { throw 'Expected two directions.' }
$data = [ordered]@{ schema_version = 1; route = '108'; period = 'present-day'; service_date = $ServiceDate.ToString('yyyy-MM-dd'); fetched_at = (Get-Date).ToUniversalTime().ToString('o'); source = $source; source_catalog = $catalog; attribution = 'ZTM Gdansk / Otwarty Gdansk - CC BY'; selection = 'Longest active stop sequence per direction on service_date, trimmed to published passenger terminals; representative trip, not a live timetable.'; directions = $directions }
New-Item "$root/data" -ItemType Directory -Force | Out-Null
$data | ConvertTo-Json -Depth 12 | Set-Content "$root/data/route_108.json" -Encoding utf8