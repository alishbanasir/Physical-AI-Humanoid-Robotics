FROM python:3.9

# Working directory set karein
WORKDIR /code

# Requirements copy aur install karein
COPY ./requirements.txt /code/requirements.txt
RUN pip install --no-cache-dir --upgrade -r /code/requirements.txt

# Pura backend ka code copy karein
COPY . /code

# Hugging Face default port 7860 use karta hai
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]