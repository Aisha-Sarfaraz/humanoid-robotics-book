from typing import List, Dict, Any
import tiktoken


class AdvancedTextSplitter:
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 100):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.encoder = tiktoken.encoding_for_model("gpt-3.5-turbo")

    def split_text(self, text: str) -> List[Dict[str, Any]]:
        """
        Split text into chunks with metadata using a simple approach
        """
        # Split text by paragraphs first
        paragraphs = text.split('\n\n')

        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            # If adding this paragraph would exceed chunk size, start a new chunk
            if len(current_chunk) + len(paragraph) > self.chunk_size and current_chunk:
                chunks.append(current_chunk.strip())
                # Add overlap by including part of the previous chunk
                if self.chunk_overlap > 0:
                    overlap_start = max(0, len(current_chunk) - self.chunk_overlap)
                    current_chunk = current_chunk[overlap_start:] + paragraph
                else:
                    current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Further split large chunks if needed
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > self.chunk_size:
                # Split large chunk into smaller pieces
                sub_chunks = self._split_large_chunk(chunk)
                final_chunks.extend(sub_chunks)
            else:
                final_chunks.append(chunk)

        chunk_data = []
        for i, chunk in enumerate(final_chunks):
            chunk_data.append({
                "content": chunk,
                "metadata": {
                    "chunk_index": i,
                    "token_count": self._token_length(chunk),
                    "char_count": len(chunk)
                }
            })

        return chunk_data

    def _split_large_chunk(self, text: str) -> List[str]:
        """
        Split a large chunk into smaller pieces
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + self.chunk_size

            # If we're not at the end, try to break at a sentence boundary
            if end < len(text):
                # Look for sentence endings near the boundary
                search_start = max(start, end - 200)  # Look back up to 200 chars
                sentence_end = -1

                for sep in ['.\n', '. ', '? ', '! ', '.\r\n', '.\r', '。\n', '。']:
                    pos = text.rfind(sep, search_start, end)
                    if pos != -1 and pos > sentence_end:
                        sentence_end = pos + len(sep)

                if sentence_end != -1:
                    end = sentence_end
                else:
                    # If no sentence boundary found, break at word boundary
                    search_start = max(start, end - 200)
                    word_end = text.rfind(' ', search_start, end)
                    if word_end != -1:
                        end = word_end

            chunks.append(text[start:end].strip())
            start = end

            # Add overlap if not at the end
            if self.chunk_overlap > 0 and start < len(text):
                start = max(start - self.chunk_overlap, 0)

        return [chunk for chunk in chunks if chunk.strip()]

    def _token_length(self, text: str) -> int:
        """
        Calculate token length using tiktoken
        """
        return len(self.encoder.encode(text))

    def validate_chunk(self, chunk: str) -> bool:
        """
        Validate if a chunk meets requirements
        """
        if len(chunk.strip()) == 0:
            return False
        if self._token_length(chunk) > self.chunk_size * 2:  # Allow some flexibility
            return False
        return True