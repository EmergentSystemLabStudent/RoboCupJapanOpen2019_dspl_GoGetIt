# 'em_follow_me' Package

The `em_follow_me` package provides real-time tracking of a human with using a particle filter and random forest.

*   Maintainer: Yuki Katsumata ([yuki.katsumata@em.ci.ritsumei.ac.jp](mailto:yuki.katsumata@em.ci.ritsumei.ac.jp)).
*   Author: Yuki Katsumata ([yuki.katsumata@em.ci.ritsumei.ac.jp](mailto:yuki.katsumata@em.ci.ritsumei.ac.jp)).

**Content:**

*   [Launch](#launch)
*   [Messages](#messages)
*   [Files](#files)

## Launch

*   Run `em_follow_me.launch` to start the `em_follow_me` action.

## Messages

*   `start`: Start `bool` message. `True` to start, `False` to stop.
*   `output`: Output `bool` message that always return `True`.

## Files

*   `em_trace_person.py`: Human tracking module. Add `tf` for `trace_target` automatically.
*   `em_follow_me_action.py`: Action for `em_follow_me`. Action name is `em_follow_me_action.py`. This module tracks the `tf` of `trace_target`.
