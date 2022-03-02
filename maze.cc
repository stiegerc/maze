#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <cassert>
#include <functional>
#include <algorithm>
#include <ostream>
#include <sstream>


// field coordinates struct
struct coords {
	int m,n;
	inline coords up() const noexcept {
		return {m-1,n};
	}
	inline coords down() const noexcept {
		return {m+1,n};
	}
	inline coords left() const noexcept {
		return {m,n-1};
	}
	inline coords right() const noexcept {
		return {m,n+1};
	}
	inline std::vector<coords> neighbors() const noexcept {
		return {up(),down(),left(),right()};
	}
};

// 2d vector class
class vec2 {
public:
	const int M, N;
	
	// ctor
	inline vec2(): vec2(0,0) {}
	inline vec2(const int M_, const int N_, const int init=0):
		M(M_), N(N_), data(M*N,init) {}
	inline vec2(const int M_, std::vector<int> data_):
		M(M_), N(M_ ? data_.size()/M: 0), data(std::move(data_)) {
		assert(data_.size()%M == 0);
	}

	// access
	inline int operator[](const coords& pos) const noexcept {
		return data[pos.m*N+pos.n];
	}
	inline int& operator[](const coords& pos) noexcept {
		return data[pos.m*N+pos.n];
	}

	// query
	inline bool empty() const noexcept {
		return data.empty();
	}
	inline bool valid(const coords& pos) const noexcept {
		return pos.m>=0 && pos.m<M && pos.n>=0 && pos.n<N;
	}
	
private:
	std::vector<int> data;
};


// function to find the number of steps map for a maze
vec2 get_steps_map(const vec2& maze, const coords& origin) {

	// check edge cases
	if (maze.empty() || !maze.valid(origin))
		return vec2();
	
	// prepare result
	vec2 res(maze.M,maze.N,-1);
	res[origin] = 0;

	// prepare current fields queue
	std::deque<coords> current_fields;
	current_fields.push_back(origin);

	// incrementally fill up the result with step numbers
	for (int current_step_index=1 ; !current_fields.empty(); ++current_step_index) {
		
		// check and fill in neighbors of current fields
		for (const auto& field: current_fields) {
			for (const auto& nn: field.neighbors()) {
				if (maze.valid(nn) && maze[nn]!=1 && res[nn]==-1) {
					res[nn] = current_step_index;
					current_fields.push_back(nn);
				}
			}
			current_fields.pop_front();
		}
	}

	return res;
}


std::vector<std::vector<coords>> find_shortest_paths(
		const vec2& map, const coords& finish) noexcept {
	
	// We start at the finish and go to the origin,
	// i.e. the only field with 0 steps. We will move only into fields
	// with a step count one below the current one, if there are multiple
	// such fields we split and recursively follow all of them to find
	// all the shortest paths across our maze from the finish back to the origin.
	// Then we reverse the order to find all the shortest paths to the finish.

	if (map.empty() || map[finish]==-1)
		return {};

	std::vector<std::vector<coords>> res;
	std::function<void(std::vector<coords>)> recv;
	recv = [&recv,&map,&res](std::vector<coords> path) -> void {
		
		while (map[path.back()]) {
			
			coords new_pos = {-1,-1};
			for (const auto& nn: path.back().neighbors()) {
				if (map.valid(nn) && map[nn]==map[path.back()]-1) {
					if (new_pos.m == -1) {
						new_pos = nn;
					} else {
						auto new_path = path;
						new_path.push_back(nn);
						recv(std::move(new_path));
					}
				}
			}
			path.push_back(new_pos);
		}

		res.push_back(std::move(path));
	};

	// call recursive lambda
	recv({finish});

	// reverse all the paths
	for (auto& path: res)
		std::reverse(res.begin(),res.end());

	return res;
}


void print_maze(const vec2& maze, std::ostream& os=std::cout) noexcept {
	
	coords pos;
	for (pos.m=0; pos.m!=maze.M; ++pos.m) {
		os << "|";
		for (pos.n=0; pos.n!=maze.N; ++pos.n)
			os << "-----|";
		os << "\n|";
		for (pos.n=0; pos.n!=maze.N; ++pos.n)
			os << "  " << (maze[pos]==1 ? "X":
				       maze[pos]==2 ? "S":
				       maze[pos]==3 ? "E": " ") << "  |";
		os << "\n";
	}
	os << "|";
	for (pos.n=0; pos.n!=maze.N; ++pos.n)
		os << "-----|";
	os << "\n";
}

void print_steps_map(const vec2& maze, const vec2& map, std::ostream& os=std::cout) noexcept {
	
	coords pos;
	for (pos.m=0; pos.m!=maze.M; ++pos.m) {
		os << "|";
		for (pos.n=0; pos.n!=maze.N; ++pos.n)
			os << "-----|";
		os << "\n|";
		for (pos.n=0; pos.n!=maze.N; ++pos.n)
			if (maze[pos]==1)
				os << "  X  |";
			else
				os << std::setw(3) << map[pos] << "  |";
		os << "\n";
	}
	os << "|";
	for (pos.n=0; pos.n!=maze.N; ++pos.n)
		os << "-----|";
	os << "\n";
}

void print_path(const vec2& maze, const std::vector<coords>& path, std::ostream& os=std::cout) noexcept {

	// print maze into a string
	std::stringstream sstr;
	print_maze(maze,sstr);
	std::string str = sstr.str();

	const int k = 6*maze.N+5;
	const int s = 12*maze.N+4;

	// mapping lambda
	const auto l = [k,s](const coords& pos) -> int {
		return k + s*pos.m + 6*pos.n;
	};

	// draw nodes
	for (const auto& pos: path)
		str[l(pos)] = 'o';

	// draw connections
	if (path.size()>=2) {
		for (int i=1; i!=path.size(); ++i) {
			
			const int dm = path[i].m - path[i-1].m;
			if (dm > 0) {
				for (int j=l(path[i-1])+s/2, e=l(path[i]); j!=e; j+=s/2)
					str[j] = '|';
			}
			if (dm < 0) {
				for (int j=l(path[i])+s/2, e=l(path[i-1]); j!=e; j+=s/2)
					str[j] = '|';
			}
			
			const int dn = path[i].n - path[i-1].n;
			if (dn > 0) {
				for (int j=l(path[i-1])+1, e=l(path[i]); j!=e; ++j)
					str[j] = '-';
			}
			if (dn < 0) {
				for (int j=l(path[i])+1, e=l(path[i-1]); j!=e; ++j)
					str[j] = '-';
			}
		}
	}

	os << str;
}


int main() {
	// maze construction guide:
	// 0: accessible field
	// 1: inaccessible field
	// 2: start
	// 3: finish
/*
	vec2 maze(4,
		{2,0,0,1,
		 0,0,0,0,
		 0,0,1,0,
		 1,0,0,3});
*/
/*
	vec2 maze(8,
		 {0,1,0,0,0,0,0,0,
		  0,1,2,1,0,1,1,0,
		  0,1,0,1,0,0,0,0,
		  0,0,0,0,0,0,1,0,
		  1,1,0,1,0,0,0,0,
		  1,0,0,0,0,1,0,0,
		  1,1,0,1,0,0,3,0,
		  0,0,0,1,0,0,0,0});
*/
/*
	vec2 maze(9,
		 {2,1,0,1,0,1,0,1,0,
		  0,0,0,0,0,0,0,0,0,
		  1,0,1,0,1,0,1,0,1,
		  0,0,0,0,0,0,0,0,0,
		  0,1,0,1,0,1,0,1,0,
		  0,0,0,0,0,0,0,0,0,
		  1,0,1,0,1,0,1,0,1,
		  0,0,0,0,0,0,0,0,0,
		  0,1,0,1,0,1,0,1,3});
*/
	vec2 maze(7,
		 {0,0,0,0,0,0,
		  0,0,0,0,1,0,
		  0,0,1,1,1,0,
		  3,0,1,2,0,0,
		  0,0,1,1,1,0,
		  0,0,0,0,1,0,
		  0,0,0,0,0,0});

	coords start;
	for (start.m=0; start.m!=maze.M; ++start.m) {
		for (start.n=0; start.n!=maze.N; ++start.n) {
			if (maze[start] == 2)
				goto lbl1; // fuck it
		}
	}
	lbl1:

	coords finish;
	for (finish.m=0; finish.m!=maze.M; ++finish.m) {
		for (finish.n=0; finish.n!=maze.N; ++finish.n) {
			if (maze[finish] == 3)
				goto lbl2; // fuck it
		}
	}
	lbl2:
	

	const auto steps_map = get_steps_map(maze,start);
	const auto paths = find_shortest_paths(steps_map,finish);

	std::cout << "maze:\n";
	print_maze(maze);
	std::cout << "\nstep map:\n";
	print_steps_map(maze,steps_map);
	std::cout << "\npaths:\n";
	
	for (const auto& path: paths) {
		print_path(maze,path);
		std::cout << "\n";
	}

	std::cout << "\nshortest path length: " << steps_map[finish];
	std::cout << "\nnumber of shortest paths: " << paths.size() << "\n";
	std::cout << "\nThank you and good night\n";

	return 0;
}
