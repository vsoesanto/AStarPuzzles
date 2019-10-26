class PriorityQueue:
    def __init__(self):
        self.q = []

    def __contains__(self, elt):
        for pair in self.q:
            if pair[0] == elt:
                return True
        return False

    def delete_min(self):
        if self.q == []:
            return []
        temp_min_pair = self.q[0]
        temp_min_value = temp_min_pair[1]
        temp_min_position = 0
        for j in range(1, len(self.q)):  # start from index 1 of q bc 0 is temp_min_pair
            if self.q[j][1] < temp_min_value:  # comparing current value with temp_min_value
                temp_min_pair = self.q[j]  # update as necessary
                temp_min_value = temp_min_pair[1]
                temp_min_position = j
        del self.q[temp_min_position]  # once min is found, remove that item from q
        return temp_min_pair  # return the value of temp_min_pair

    def insert(self, state, priority):
        if self[state] != -1:
            print("Error: You're trying to insert an element into a My_Priority_Queue instance,")
            print(" but there is already such an element in the queue.")
            return
        self.q.append((state, priority))

    def __len__(self):
        return len(self.q)

    def __getitem__(self, state):
        '''This method enables Pythons right-bracket syntax.
        Here, something like  priority_val = my_queue[state]
        becomes possible. Note that the syntax is actually used
        in the insert method above:  self[state] != -1  '''
        for (S, P) in self.q:
            if S == state: return P
        return -1  # This value means not found.

    def __delitem__(self, state):
        '''This method enables Python's del operator to delete
        items from the queue.'''
        for count, (S, P) in enumerate(self.q):
            if S == state:
                del self.q[count]
                return

    def __str__(self):
        txt = "PRIORITY QUEUE: ["
        for (s, p) in self.q: txt += '(' + str(s) + ',' + str(p) + ') '
        txt += ']'
        return txt