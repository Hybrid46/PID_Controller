using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneSelector : MonoBehaviour {
    [SerializeField]
    int currentScene;

    public void LoadNextLevel() {
        SceneManager.LoadScene(currentScene + 1);
    }

    public void LoadLastLevel() {
        SceneManager.LoadScene(currentScene - 1);
    }
}
