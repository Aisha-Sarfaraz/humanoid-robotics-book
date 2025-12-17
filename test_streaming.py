"""
Test Streaming Endpoint
"""
import requests
import json
import time

def test_streaming_endpoint():
    """Test the streaming chat endpoint"""

    url = "http://127.0.0.1:8000/api/v1/chat/stream"

    payload = {
        "message": "What is ROS 2?",
        "session_id": f"test-{int(time.time())}",
        "selected_text": None
    }

    print("=" * 70)
    print("TESTING STREAMING ENDPOINT")
    print("=" * 70)
    print(f"\nEndpoint: {url}")
    print(f"Query: {payload['message']}")
    print("\nStreaming Response:")
    print("-" * 70)

    try:
        response = requests.post(
            url,
            json=payload,
            stream=True,
            headers={"Accept": "text/event-stream"}
        )

        if response.status_code != 200:
            print(f"[ERROR] Status code: {response.status_code}")
            print(response.text)
            return

        full_response = ""
        sources = []

        for line in response.iter_lines():
            if line:
                line = line.decode('utf-8')

                if line.startswith('event:'):
                    event_type = line.split(':', 1)[1].strip()
                elif line.startswith('data:'):
                    data = json.loads(line.split(':', 1)[1].strip())

                    if event_type == 'status':
                        print(f"\n[STATUS] {data['message']}")

                    elif event_type == 'sources':
                        sources = data['sources']
                        print(f"\n[SOURCES] Found {data['count']} relevant documents:")
                        for i, source in enumerate(sources, 1):
                            print(f"  {i}. {source['title']} (score: {source['score']:.3f})")

                    elif event_type == 'chunk':
                        content = data['content']
                        full_response += content
                        print(content, end='', flush=True)

                    elif event_type == 'done':
                        print(f"\n\n[DONE] Session: {data['session_id']}")
                        print(f"       Response length: {data['total_length']} chars")
                        print(f"       Sources used: {data['sources_used']}")

                    elif event_type == 'error':
                        print(f"\n[ERROR] {data['message']}")

        print("\n" + "-" * 70)
        print("[SUCCESS] Streaming test completed!")
        print(f"\nFull response ({len(full_response)} chars):")
        print(full_response)

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 70)


if __name__ == "__main__":
    test_streaming_endpoint()
