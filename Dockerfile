FROM python:3.12-slim
WORKDIR /bench
COPY . .
RUN python -m unittest discover -s tests
ENTRYPOINT ["python", "run.py"]
CMD ["--full"]
