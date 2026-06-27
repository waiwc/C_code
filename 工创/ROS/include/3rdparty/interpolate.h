	  template <typename T>
	  T Interpolate(T prev, T next, double lambda) {
		  return (1 - lambda) * prev + lambda * next;
	  }

