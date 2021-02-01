/**
 * Definition of the map:
 * The map is a 4 x 4 grid world. The position of the grid location is 
 * encoded as [X,Y], where the bottom left corner is [0,0] and the top right 
 * corner is [3,3].
 *
 * The map has accessible and inaccessible squares. A visual view of the map is
 * below. The inaccessible square have an X. (start without that)
 *
* |-------------------------------|
* |   m   |   n   |   o   |   p   |
* |       |       |       |   X   |
* | [0,0] | [1,0] | [2,0] | [3,0] |
* |-------------------------------|
* |   i   |   j   |   k   |   l   |
* |       |   X   |       |       |
* | [0,1] | [1,1] | [2,1] | [3,1] |
* |-------------------------------|
* |   e   |   f   |   g   |   h   |
* |       |   X   |       |       |
* | [0,2] | [1,2] | [2,2] | [3,2] |
* |-------------------------------|
* |   a   |   b   |   c   |   d   |
* |       |       |   X   |       |
* | [0,3] | [1,3] | [2,3] | [3,3] |
* |-------------------------------|
*/

locationName(a,[0,3]).
locationName(b,[1,3]).
locationName(c,[2,3]).
locationName(d,[3,3]).
locationName(e,[0,2]).
locationName(f,[1,2]).
locationName(g,[2,2]).
locationName(h,[3,2]).
locationName(i,[0,1]).
locationName(j,[1,1]).
locationName(k,[2,1]).
locationName(l,[3,1]).
locationName(m,[0,0]).
locationName(n,[1,0]).
locationName(o,[2,0]).
locationName(p,[3,0]).
 
// Possible map transitions.
// possible(StartingPosition, PossibleNewPosition)
possible(a,b).
possible(a,e).

possible(b,a).

possible(d,h).

possible(e,a).
possible(e,i).

possible(g,h).
possible(h,d).
possible(h,l).

possible(i,e).
possible(i,m).

possible(k,g).
possible(k,l).
possible(k,o).

possible(l,k).
possible(l,h).

possible(m,i).
possible(m,n).

possible(n,m).
possible(n,o).

possible(o,n).
possible(o,k).


// Mess with the map, tell that C is a possible place to go
possible(b,c).
possible(c,d).

