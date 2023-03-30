This package is a C++ porting of the Simple Commander API provided with Nav2. See https://navigation.ros.org/commander_api/ for a more thorough discussion of "navigation as a library". This porting targets the Foxy distribution.

<b>Porting notes</b></br>
The C++ <code>Navigator</code> class is the equivalent of the python <code>BasicNavigator</code> class. The following methods have been ported and offer the same functionalities (modulo some minor changes described below). 

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
<td>setInitialPose</td>
</tr>
<tr>
<td>goToPose</td>
<td>goToPose</td>
</tr>
<tr>
<td>followWayPoints</td>
<td>followWayPoints</td>
</tr>
<tr>
<td>spin</td>
<td>spin</td>
</tr>
<tr>
<td>backup</td>
<td>backup</td>
</tr>
<tr>
<td>followPath</td>
<td>followPath</td>
</tr>
<tr>
<td>cancelTask</td>
<td>cancelTask</td>
</tr>
<tr>
<td>isTaskComplete</td>
<td>isTaskComplete</td>
</tr>
<tr>
<td>getFeedback</td>
<td>getFeedback</td>
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
<td>changeMap</td>
</tr>
<tr>
<td>clearAllCostMaps</td>
<td>clearAllCostMaps</td>
</tr>
<tr>
<td>clearLocalCostMap</td>
<td>clearLocalCostMap</td>
</tr>
<tr>
<td>clearGlobalCostMap</td>
<td>clearGlobalCostMap</td>
</tr>
<tr>
<td>getGlobalCostmap</td>
<td>getGlobalCostmap</td>
</tr>
<tr>
<td>getLocalCostmap</td>
<td>getLocalCostmap</td>
</tr>
<tbody>
</table>






API Differences<br>
<ul>
<li> <code>CancelTask</code> is a blocking method, i.e., if a task is being executed and the server accepts the cancellation request, the method does not return until the task is canceled by the server.
<li> <code>GetFeedback</code> returns a generic pointer <code>std::shared_ptr < const void ></code>   that must be cast to the appropriat type to access the feedback message. See the <code>testpackage.cpp</code> for an example of how this is done.
</ul>

The following methods have <i>not</i> been ported: <code>goThroughPose</code>, <code>assistedTeleop</code>, <code>getPathThroughPoses</code>, <code>smoothPath</code>.
