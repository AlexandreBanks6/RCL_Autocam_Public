import numpy as np
import PyKDL

class CircularBuffer:
    def __init__(self, size, num_elements):
        self.size = size
        self.buffer = [None] * size
        self.index = 0
        self.is_full = False
        self.num_elements = num_elements

    def append(self, element):
        if not isinstance(element, np.ndarray) or element.shape != (self.num_elements,):
            raise ValueError("Element must be a numpy array of length " + str(self.num_elements))
        
        self.buffer[self.index] = element
        self.index = (self.index + 1) % self.size
        if self.index == 0:
            self.is_full = True

    def get_buffer(self):
        if not self.is_full:
            return self.buffer[:self.index]
        else:
            return self.buffer[self.index:] + self.buffer[:self.index]
    
    def get_mean(self):
        """
        Calculate the mean of the elements currently in the buffer.

        Returns:
            np.array: The mean of the elements in the buffer.
        """
        current_buffer = self.get_buffer()
        
        if not current_buffer:
            return np.zeros(self.num_elements)  # Return zeros if the buffer is empty
        
        return np.mean(current_buffer, axis=0)
    
    def negateQuaternion(self,quat): 
        if quat[3] < 0:
            quat = -1*quat
        return quat
    

class rotationBuffer(CircularBuffer):
    def __init__(self, size, num_elements):
        super().__init__(size, num_elements) #call parent constructor

    #recieves a pyKDL rotation frame (.M)
    def append(self, element):
        quat = element.GetQuaternion()
        quat = np.array([quat[0],quat[1], quat[2], quat[3]])
        quat = self.negateQuaternion(quat)
        super().append(quat)
    
    #@override method (returns a .M pykdl frame)
    def get_mean(self):
        quat = super().get_mean()
        quat = self.negateQuaternion(quat)
        return PyKDL.Rotation.Quaternion(quat[0],quat[1],quat[2],quat[3])

    
