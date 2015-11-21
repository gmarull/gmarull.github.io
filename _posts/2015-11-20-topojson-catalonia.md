---
layout: post
title: TopoJSON maps for Catalonia
---

After trying some of the [TopoJSON](//github.com/mbostock/topojson)
[examples](//github.com/mbostock/topojson/wiki/Gallery) I could not resist the
temptation of learning more about it. As a Catalan citizen, I decided to focus
on [Catalonia](//en.wikipedia.org/wiki/Catalonia) (or *Catalunya* in Catalan).
Before starting I did a quick search to see if somebody else had already used
TopoJSON in my region. I found some nice examples from a couple of authors:
[martgnz](//bl.ocks.org/martgnz) and [rveciana](//bl.ocks.org/rveciana). However
in these examples little information is given on how maps are generated.
Particularly topics like map simplification or projection are not discussed. So
I decided to write this post where I explain in some detail the process of
generating the TopoJSON maps for Catalonia.

> Together with this post I have created the
> [cat-topojson](//github.com/teslabs/cat-topojson) utility which will allow you
> to easily generate the maps.

<script src="//d3js.org/d3.v3.min.js" charset="utf-8"></script>
<script src="//d3js.org/topojson.v1.min.js"></script>

<style>

.land {
  fill: white;
  transition: fill 0.4s ease;
}

.land:hover {
  fill: #57ad68;
}

.border-in {
  fill: none;
  stroke: black;
  stroke-linejoin: round;
  stroke-linecap: round;
}

.border-out {
  fill: none;
  stroke: black;
  stroke-width: 2;
  stroke-linejoin: round;
  stroke-linecap: round;
}

</style>

<script>

var width  = 500,
    height = 500;

var path = d3.geo.path()
      .projection(null);

d3.json("{{ site.url }}/content/posts/topojson-catalonia/cat-comarques.json", function(error, cat) {
  if (error) throw error;

  var svg = d3.select("div#map-example").append("svg")
    .attr("width", width)
    .attr("height", height);

  svg.append("g")
    .selectAll("path")
      .data(topojson.feature(cat, cat.objects.comarques).features)
    .enter().append("path")
      .attr("d", path)
      .attr("class", "land")
    .append("title")
      .text(function(d) { return d.properties.nom + ": " +
                                 d.properties.sup + " km²"; });

  svg.append("path")
    .datum(topojson.mesh(cat, cat.objects.comarques,
        function (a, b) { return a !== b; }))
    .attr("class", "border-in")
    .attr("d", path);

  svg.append("path")
    .datum(topojson.mesh(cat, cat.objects.comarques,
        function (a, b) { return a === b; }))
    .attr("class", "border-out")
    .attr("d", path);
});

</script>

<figure>
  <div id="map-example" class="text-center"></div>
  <figcaption>
    Example: hover the mouse on any county to know its area!
  </figcaption>
</figure>

# Required tools

The first requirement is the TopoJSON host tool, ``topojson``, which in turn
requires [Node.js](//nodejs.org/en/). If you are on OS X, and assuming you use
[brew](//brew.sh/), you can easily get both by typing:

{% highlight bash %}
$ brew install node
$ npm install -g topojson
{% endhighlight %}

In this post we will also use some tools part of [GDAL](//www.gdal.org/), useful
to convert from one coordinate system to another or simply to retrieve
information from map files. It can also be installed using ``brew``:

{% highlight bash %}
$ brew install gdal
{% endhighlight %}

# Source maps

Before we proceed further, we need to obtain the source maps. They are freely
available from [ICGC](//www.icc.cat/vissir3) (registration is required).  The
maps of interest are called *Base Municipal* and are offered in multiple scales
(1:50, 1:250 and 1:1000) and formats (DXF, DGN and SHP).  In the examples found
in this post we will use the scale 1:50 and the format SHP (shapefile), which is
the only one of the list supported by ``topojson``. The specification for these
maps can be found
[here](//www.icc.es/cat/Home-ICC/Digital-geoinformation/About-ICGC-geoinformation/Technical-specifications).
A summary of the relevant information is given below.

The shapefiles of interest contained in the downloaded file are the following:

<table>
    <thead>
        <tr> <th>File name</th> <th>Description</th> </tr>
    </thead>
    <tbody>
        <tr>
            <td><code>bm[eeee]mv33sh1fpm[m]_[aaaammdd]_[c].shp</code></td>
            <td>Municipality Polygons</td>
        </tr>
        <tr>
            <td><code>bm[eeee]mv33sh1fpc[m]_[aaaammdd]_[c].shp</code></td>
            <td>County Polygons</td>
        </tr>
        <tr>
            <td><code>bm[eeee]mv33sh1fpp[m]_[aaaammdd]_[c].shp</code></td>
            <td>Province Polygons</td>
        </tr>
    </tbody>
</table>

where ``eeee`` is the scale (i.e., ``50``, ``250``, ``1000``), ``m`` is the
frame coordinate reference (only ``1`` is available, meaning EPSG:25831 - ETRS89
/ UTM zone 31N), ``aaaammdd`` is the maps reference date (e.g. ``20150501``) and
``c`` is the distribution revision (e.g. ``0``). Other files are provided, e.g.
the location of municipality capital cities, but they will not be analyzed in
this post.

We can easily get detailed information of each file using the ``ogrinfo``
utility provided by GDAL, e.g.

{% highlight bash %}
$ ogrinfo -al bm50mv33sh1fpm1_20150501_0.shp | less
Geometry: Polygon
Feature Count: 947
Extent: (260188.973378, 4488778.587563) - (527401.970714, 4747980.912187)
Layer SRS WKT:
PROJCS["ETRS_1989_UTM_Zone_31N",
    GEOGCS["GCS_ETRS_1989",
        DATUM["European_Terrestrial_Reference_System_1989",
            SPHEROID["GRS_1980",6378137.0,298.257222101]],
        PRIMEM["Greenwich",0.0],
        UNIT["Degree",0.0174532925199433]],
    PROJECTION["Transverse_Mercator"],
    PARAMETER["False_Easting",500000.0],
    PARAMETER["False_Northing",0.0],
    PARAMETER["Central_Meridian",3.0],
    PARAMETER["Scale_Factor",0.9996],
    PARAMETER["Latitude_Of_Origin",0.0],
    UNIT["Meter",1.0]]
MUNICIPI: String (6.0)
COMARCA: String (2.0)
PROVINCIA: String (2.0)
NOM_MUNI: String (45.0)
NOMN_MUNI: String (45.0)
NOMG_MUNI: String (45.0)
CAP_MUNI: String (30.0)
CAPN_MUNI: String (30.0)
CAPG_MUNI: String (30.0)
SUP_MUNI: Real (6.2)
ORSUP_MUNI: String (1.0)
...
{% endhighlight %}

# Generation of the TopoJSON maps

In the following sections the process of generating the TopoJSON maps from the
ICGC shapefiles is discussed.

## Geometry

There are [two
types](//resources.arcgis.com/en/help/main/10.1/index.html#//00v20000000q000000)
of coordinate systems used in maps:
[projected](//resources.arcgis.com/en/help/main/10.1/index.html#/What_are_projected_coordinate_systems/003r0000000p000000/)
(X, Y) or
[geographic](//resources.arcgis.com/en/help/main/10.1/index.html#/What_are_geographic_coordinate_systems/003r00000006000000/)
($$\lambda$$, $$\varphi$$). As we have just seen in the previous section, the
maps provided by ICGC are projected in
[UTM](//resources.arcgis.com/en/help/main/10.1/index.html#/Universal_Transverse_Mercator/003r00000049000000/).
Advantages of using projected coordinates include better simplification results
as well as reduced overhead on the rendering process as stated in this
[example](//bl.ocks.org/mbostock/5557726). You can use either one or the other
with D3.js, so choose what best fits your application. In this post examples for
both cases are provided.

In order to change from one coordinate system to another, you can use the
``ogr2ogr`` utility provided by GDAL. Actually, ``ogr2ogr`` offers many more
possibilities (e.g. filtering), but it is left to the reader to explore them. As
a quick example, if you want to convert the counties shapefile to the [World
Geodetic System](//en.wikipedia.org/wiki/World_Geodetic_System),
[EPSG:4326](//spatialreference.org/ref/epsg/4326/):

{% highlight bash %}
$ ogr2ogr \
    -f 'ESRI Shapefile' \
    -t_srs EPSG:4326 \
    counties-wgs.shp \
    bm50mv33sh1fpc1_20150501_0.shp
{% endhighlight %}

Provided that your input maps are in geographical coordinates, you could also
use the ``topojson --projection`` option to apply one of the [D3.js
projections](//github.com/mbostock/d3/wiki/Geo-Projections).

Finally, when using projected coordinates, we can specify ``--width``,
``--height`` and ``--margin`` to fit a viewport of the specified size. This will
simplify the map usage if we know in advance where we are going to display the
map, as we will not need to translate or scale it (see the examples).

## Simplification and quantization

An important thing to care about when distributing maps is its complexity and
size, both intrinsically related. Two techniques can be used to reduce both:
geometry simplification and quantization.

As illustrated, in this [article](//bost.ocks.org/mike/simplify/) geometry
simplification reduces the number of points given an area threshold. This area
threshold is specified through the ``-s`` argument. It is given in
[steradians](//en.wikipedia.org/wiki/Steradian) in geographical coordinates. In
case of projected coordinates it is given as squared base units, e.g. if we use
``--width`` and/or ``--height`` it will be in px<sup>2</sup>.

Quantization is about numeric precision. TopoJSON uses [fixed
point](//www.digitalsignallabs.com/fp.pdf) coordinates, and allows you to
specify the number of differentiable values. It is controlled by the ``-q``
parameter, or ``--pre-quantization`` and ``--post-quantization`` to specify
input and output quantizations, respectively. A value of 10.000 differentiable
values is used by default, which in most cases will be fine. Roughly speaking,
the more values, the larger the size of the map.

In any case, the simplest way to choose the right simplification and
quantization parameters is to try and see how map looks. Three illustrative
examples are shown below, exaggerating the effects of simplification and
quantization in the second and third maps, respectively.

<div class="grid">
    <div class="grid-col-33" id="map-simpl-1"></div>
    <div class="grid-col-33" id="map-simpl-2"></div>
    <div class="grid-col-33" id="map-quant"></div>
</div>

<style>

.sq-land {
  fill: white;
  stroke: black;
  stroke-width: 0.5px;
}

</style>

<script>

d3.json("{{ site.url }}/content/posts/topojson-catalonia/cat-comarques-simpl-1.json", function(error, cat) {
  if (error) throw error;

  var path = d3.geo.path()
      .projection(null);

  var svg = d3.select("div#map-simpl-1").append("svg")
    .attr("width", 250)
    .attr("height", 250);

  svg.append("path")
    .datum(topojson.feature(cat, cat.objects.comarques))
    .attr("class", "sq-land")
    .attr("d", path);
});

d3.json("{{ site.url }}/content/posts/topojson-catalonia/cat-comarques-simpl-2.json", function(error, cat) {
  if (error) throw error;

  var path = d3.geo.path()
    .projection(null);

  var svg = d3.select("div#map-simpl-2").append("svg")
    .attr("width", 250)
    .attr("height", 250);

  svg.append("path")
    .datum(topojson.feature(cat, cat.objects.comarques))
    .attr("class", "sq-land")
    .attr("d", path);
});

d3.json("{{ site.url }}/content/posts/topojson-catalonia/cat-comarques-quant.json", function(error, cat) {
  if (error) throw error;

  var path = d3.geo.path()
    .projection(null);

  var svg = d3.select("div#map-quant").append("svg")
    .attr("width", 250)
    .attr("height", 250);

  svg.append("path")
    .datum(topojson.feature(cat, cat.objects.comarques))
    .attr("class", "sq-land")
    .attr("d", path);
});

</script>

## Properties

The ICGC shapefiles contain some properties which are useful to retain (e.g.
province names, area, etc.). They are detailed in the specification file.

By default, ``topojson`` does not copy the properties contained in the input
files. The ``-p`` argument can be used to copy all properties, or ``-p
target=source`` to only keep the ``source`` property while renaming it to
``target``. More than one property can be copied by passing multiple ``-p``
arguments or by appending multiple properties, e.g. ``-p
name=NAME,surname=SURN``. Furthermore, prepending a ``+`` sign to a property
name will force the property to be a number, e.g. ``-p area=+AREA``.

Another important argument is ``--id-property``. It is used to assign any of the
properties to the ID of the geometry.

# Examples

Finally to conclude this post a couple of minimal examples are provided. Note
that not many details are given on how to use D3.js/TopoJSON, as it is not the
aim of this post. If you want to learn more about the topic, you can find great
tutorials online like [this](//bost.ocks.org/mike/map/).

In order to run the examples you will need an HTTP server, e.g.

{% highlight bash %}
$ python -m SimpleHTTPServer
Serving HTTP on 0.0.0.0 port 8000 ...
{% endhighlight %}

## Projected map

In this example we will use the counties (*comarques*) map and fit it to a
viewport of 500x500 px. As the source map is already projected, we can just feed
it into ``topojson``:

{% highlight bash %}
$ topojson \
    -o cat-comarques.json \
    --width=500 --height=500 \
    --simplify=2 \
    --id-property=+COMARCA \
    -p nom=NOM_COMAR \
    -p cap=CAP_COMAR \
    -p sup=SUP_COMAR \
    -- comarques=bm50mv33sh1fpc1_20150501_0.shp
{% endhighlight %}

Then, we can quickly visualize the results creating an html file with the
following code:

{% highlight html %}
<!DOCTYPE html>
<meta charset="utf-8">
<style>

.land {
  fill: #ddc;
  stroke: white;
}

</style>

<body>
<script src="//d3js.org/d3.v3.min.js" charset="utf-8"></script>
<script src="//d3js.org/topojson.v1.min.js"></script>

<script>

var width  = 500,
    height = 500;

// no need to project!
var path = d3.geo.path()
    .projection(null);

var svg = d3.select("body").append("svg")
    .attr("width", width)
    .attr("height", height);

d3.json("cat-comarques.json", function(error, cat) {
  if (error) throw error;

  svg.append("path")
      .datum(topojson.feature(cat, cat.objects.comarques))
      .attr("d", path)
      .attr("class", "land")
});

</script>
{% endhighlight %}

...and voilà:

<iframe src="{{ site.url }}/content/posts/topojson-catalonia/example-1.html" width="500" height="500" marginwidth="0" marginheight="0" scrolling="no"></iframe>
[Open in a new window]({{ site.url }}/content/posts/topojson-catalonia/example-1.html){:target="_blank"}

## Non-projected map

In this second example we will use the municipalities (*municipis*) map but the
projection will be done on the browser. We first use ``ogr2ogr`` to convert our
map to the [WGS-84](//spatialreference.org/ref/epsg/4326/) geographical
coordinate system:

{% highlight bash %}
$ ogr2ogr \
    -f 'ESRI Shapefile' \
    -t_srs EPSG:4326 \
    municipis-wgs.shp \
    bm50mv33sh1fpm1_20150501_0.shp
{% endhighlight %}

Then, we use ``topojson`` to generate the map:

{% highlight bash %}
$ topojson \
    -o cat-municipis.json \
    --simplify=1e-8 \
    --id-property=+MUNICIPI \
    -p nom=NOM_MUNI \
    -p comarca=+COMARCA \
    -p provincia=+PROVINCIA \
    -p sup=SUP_MUNI \
    -- municipis=municipis-wgs.shp
{% endhighlight %}

Note that an area threshold of 10<sup>-8</sup> sr has proved to deliver a good
compromise after a few tries.

When using a projected map already adjusted to a viewport we do not need to care
about centering or scaling it, but now we do. Finding the right values is a
tedious task, as you need to go through a trial and error process until you find
an acceptable result. However, there is an easy way to automatically center and
scale a map described in this great [example](//bl.ocks.org/mbostock/4707858),
which could even be used to calculate the right static values. Summarizing, the
example code looks like this:

{% highlight html %}
<!DOCTYPE html>
<meta charset="utf-8">
<style>

.land {
  fill: #ddc;
  stroke: white;
}

</style>

<body>
<script src="//d3js.org/d3.v3.min.js" charset="utf-8"></script>
<script src="//d3js.org/topojson.v1.min.js"></script>

<script>

var width  = 500,
    height = 500;

var projection = d3.geo.mercator();

var path = d3.geo.path()
    .projection(projection);

var svg = d3.select("body").append("svg")
    .attr("width", width)
    .attr("height", height);

d3.json("cat-municipis.json", function(error, cat) {
  if (error) throw error;

  var municipis = topojson.feature(cat, cat.objects.municipis);

  // automatic center and scale (see http://bl.ocks.org/mbostock/4707858)
  projection
      .scale(1)
      .translate([0, 0]);

  var b = path.bounds(municipis),
      s = .95 / Math.max((b[1][0] - b[0][0]) / width, (b[1][1] - b[0][1]) / height),
      t = [(width - s * (b[1][0] + b[0][0])) / 2, (height - s * (b[1][1] + b[0][1])) / 2];

  projection
      .scale(s)
      .translate(t);

  svg.append("path")
      .datum(municipis)
      .attr("class", "land")
      .attr("d", path);

});

</script>
{% endhighlight %}

...and produces:

<iframe src="{{ site.url }}/content/posts/topojson-catalonia/example-2.html" width="500" height="500" marginwidth="0" marginheight="0" scrolling="no"></iframe>
[Open in a new window]({{ site.url }}/content/posts/topojson-catalonia/example-2.html){:target="_blank"}

---

**NOTE:** Source maps are produced by ICGC and are subject to [terms of
use](//www.icc.cat/conditions).

