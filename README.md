# DARPA Sprint 3: HST Data Analysis

The excitement_time branch code base parses data from Sprint 3 HST experiments to gather excitement times. Primary goal is to record all the time stamps at which a player performed an action that might excite them in some way. These time stamps will be compared to determine if the excitement can be interpreted by changes in biometric data. In particular, times or the following events will be recorded

- When a player gives a trajectory command to a drone. Note that this happens only in following control types = ['waypoint', 'directergodic', 'sharedergodic].
- When a player looses a life.
- When a player finds a treasure.

## Getting Started

To successfully run this code, path to the saved rosbag files needs to be updated. This can be done in line 70 of the *save_date.py* script. Once this is updated, to perform this analysis one needs to be in the *src* folder of the package and run *save_data.py* code as following:

```
cd ~/darpa_dataanalysis/src
python save_data.py
```
in terminal window.

## Built With

* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2) - The README.md template


## Versioning

August 17, 2020
* In performance.py
  * **line 19:** added topic */input* in order to get information about trajectories and drones
  * **line 38:** under *reset_game* added variables *self.time_XXX* in order to track list of all the important times
  * **line 91:**  updating *time_treasure_found*
  * **line 130:** updating *time_player_input*
  * **line 152:** concatenating and sorting all time stamps into the final *time_excitement* list
 * **line 214:** updating *time_life_lost*
* In save_data.py
  * **line 21:** added *excitement_data.csv* to record *time_excitement* for all the subjects & trials
  * **line 70:** updated path to all the saved rosbag files
  * **line 105:** saving data in one *.csv* file
  * **line 112:** saving data under folder *data* for each subject and control individually

## Authors

* Millicent Schlafly
* Katarina Popovic

## License

This project is licensed under the BSD License - see the [LICENSE.md](LICENSE.md) file for details
