$Executable = "A:\University\CSCI520\Homework2\csci520-assignment2\IDE-starter\VS2017\Debug\interpolate.exe"

$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131-dance.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131_04-dance.amc l e 20 131_04-dance-linear-euler.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131-dance.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131_04-dance.amc b e 20 131_04-dance-bezier-euler.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131-dance.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131_04-dance.amc l q 20 131_04-dance-linear-quaternion.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131-dance.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\131_04-dance.amc b q 20 131_04-dance-bezier-quaternion.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135-martialArts.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135_06-martialArts.amc l e 40 135_06-martialArts-linear-euler.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135-martialArts.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135_06-martialArts.amc b e 40 135_06-martialArts-bezier-euler.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135-martialArts.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135_06-martialArts.amc l q 40 135_06-martialArts-linear-quaternion.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms


$Parms = "A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135-martialArts.asf A:\University\CSCI520\Homework2\csci520-assignment2\mocapPlayer-starter\135_06-martialArts.amc b q 40 135_06-martialArts-bezier-quaternion.amc"
$Parms = $Parms.Split(" ")

& "$Executable" $Parms