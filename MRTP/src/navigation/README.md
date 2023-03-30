This package is a C++ porting of the Simple Commander API provided with Nav2 and implemented in Python. See https://navigation.ros.org/commander_api/ for a more thorough discussion of "navigation as a library". This porting targets the Foxy distribution. Whenever possible, we follow the same API and also the internal implementation of the original Python implementation. 

<b>Porting notes</b></br>
The C++ <code>Navigator</code> class is the equivalent of the Python <code>BasicNavigator</code> class. The following methods have been ported and offer the same functionalities (modulo some minor changes described below). 

<table>
<thead>
<tr>
<td>Python method</td>
<td>C++ method</td>
</tr>
</thead>
<tbody>
<tr>
<td>setInitialPose</td>
<td>SetInitialPose</td>
</tr>
<tr>
<td>goToPose</td>
<td>GoToPose</td>
</tr>
<tr>
<td>followWayPoints</td>
<td>FollowWaypoints</td>
</tr>
<tr>
<td>spin</td>
<td>Spin</td>
</tr>
<tr>
<td>backup</td>
<td>Backup</td>
</tr>
<tr>
<td>followPath</td>
<td>FollowPath</td>
</tr>
<tr>
<td>cancelTask</td>
<td>CancelTask</td>
</tr>
<tr>
<td>isTaskComplete</td>
<td>IsTaskComplete</td>
</tr>
<tr>
<td>getFeedback</td>
<td>GetFeedback</td>
</tr>
<tr>
<td>getResult</td>
<td>getResult</td>
</tr>
<tr>
<td>waitUntilNav2Active</td>
<td>waitUntilNav2Active</td>
</tr>
<tr>
<td>getPath</td>
<td>getPath</td>
</tr>
<tr>
<td>changeMap</td>
<td>ChangeMap</td>
</tr>
<tr>
<td>clearAllCostmaps</td>
<td>ClearAllCostmaps</td>
</tr>
<tr>
<td>clearLocalCostmap</td>
<td>ClearLocalCostmap</td>
</tr>
<tr>
<td>clearGlobalCostmap</td>
<td>ClearGlobalCostmap</td>
</tr>
<tr>
<td>getGlobalCostmap</td>
<td>GetGlobalCostmap</td>
</tr>
<tr>
<td>getLocalCostmap</td>
<td>GetLocalCostmap</td>
</tr>
<tbody>
</table>

Internal methods (those with the name starting with <code>_</code>) have either been ported as private methods, or not ported (or implemented differently). Either way, they are not available.

API Differences<br>
<ul>
<li> <code>CancelTask</code> is a blocking method, i.e., if a task is being executed and the server accepts the cancellation request, the method does not return until the task is canceled by the server.
<li> <code>GetFeedback</code> returns a generic pointer <code>std::shared_ptr < const void ></code>   that must be cast to the appropriat type to access the feedback message. See the <code>testpackage.cpp</code> for an example of how this is done.
</ul>

The following methods have <i>not</i> been ported because the corresponding action servrs are not part of the Foxy distribution: <code>goThroughPose</code>, <code>assistedTeleop</code>, <code>getPathThroughPoses</code>, <code>smoothPath</code>.

  The file <code>testpackage.cpp</code> shows how each method can be called. To test it, follow the same Gazebo/RViz setup described in https://navigation.ros.org/getting_started/index.html
