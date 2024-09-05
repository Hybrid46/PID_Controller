using UnityEngine;

public class TargetButton : MonoBehaviour {
    [SerializeField]
    Controller controller;
    [SerializeField]
    int value;

    public void SetValue() {
        controller.SetTarget(value);
    }
}
