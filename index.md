---
layout: default
permalink: /
---

# Welcome!

![Author]({{ site.url }}/content/home/author.jpg){: .pull-right style="width: 31%"}

My name is Gerard Marull-Paretas and I am a
[Catalan](http://en.wikipedia.org/wiki/Catalonia) [Telecommunications
Engineer](http://en.wikipedia.org/wiki/Telecommunications_engineering) currently
in the Electrical Engineering MS/PhD program at the [University of California,
Irvine](http://www.eng.uci.edu). This site serves as a place to occasionally
write some articles as well as to display my [CV](/cv/).  If you are wondering
what is the figure spinning on top, read
[this](http://en.wikipedia.org/wiki/Lissajous_curve).

### Find me on...

* [Facebook](//facebook.com/gmarullp)
* [LinkedIn](//uk.linkedin.com/in/gmarullp)
* [Github](//github.com/teslabs)
* [bl.ocks.org](//bl.ocks.org/teslabs)

{% assign total_posts = site.posts | size %}
{% if total_posts >= 1 %}
## Latest articles
<ul>
  {% for post in site.posts limit: 5 %}
  <li>
    <a href="{{ post.url | prepend: site.baseurl }}">{{ post.title }}</a>
    <span class="post-date">{{ post.date | date: "%b %-d, %Y" }}</span>
  </li>
  {% endfor %}
</ul>
{% endif %}
