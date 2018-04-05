---
layout: default
permalink: /
---

# Welcome!

Teslabs Engineering is a small engineering consultancy owned by Gerard
Marull-Paretas, a Software and Electrical Engineer. It is located at the
Barcelona area offering Software and Electrical Engineering services. You can
check what we can offer on the [Services](/services) page. We offer on-site
services as well as remote work.

{% assign total_posts = site.posts | size %}
{% if total_posts >= 1 %}
## Latest technical articles
<ul>
  {% for post in site.posts limit: 5 %}
  <li>
    <a href="{{ post.url | prepend: site.baseurl }}">{{ post.title }}</a>
    <span class="post-date">{{ post.date | date: "%b %-d, %Y" }}</span>
  </li>
  {% endfor %}
</ul>
{% endif %}
