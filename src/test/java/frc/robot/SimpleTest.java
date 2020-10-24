import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class SimpleTest {
    private List<Integer> list;

    @Before
    public void setUp() {
        list = new ArrayList<>();
        list.add(3);
        list.add(1);
        list.add(4);
        list.add(1);
        list.add(5);
        list.add(9);
    }

    @After
    public void tearDown() {
        list.clear();
    }

    @Test
    public void shouldBeOkToAlterTestData() {
        list.remove(0); // Remove first element of list.
        assertEquals(4, list.size()); // Size is down to five
    }

    @Test
    public void shouldBeIndependentOfOtherTests() {
        assertEquals(6, list.size());
    }    
}
